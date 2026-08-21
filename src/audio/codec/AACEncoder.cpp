#include "audio/codec/AACEncoder.hpp"

#if defined(USE_AAC) && USE_AAC
#include "config/Config.hpp"
#include "util/Logger.hpp"
#include <cstdint>
#include <cstring>
#include <ctime>

AACEncoder *AACEncoder::createNew(int sampleRate, int numChn) {
  return new AACEncoder(sampleRate, numChn);
}

AACEncoder::AACEncoder(int sampleRate, int numChn)
    : sampleRate(sampleRate), numChn(numChn) {}

AACEncoder::~AACEncoder() {
  close();
}

int AACEncoder::open() {
  faac_params params;
  faac_status st = faac_params_init(&params);
  if (st != FAAC_OK) {
    LOG_ERROR("faac_params_init failed: " << faac_strerror(st));
    return -1;
  }

  params.sample_rate = sampleRate;
  params.num_channels = numChn;
  params.mpeg_version = FAAC_MPEG4;
  params.object_type = FAAC_OBJ_LOW;
  params.input_format = FAAC_INPUT_16BIT;
  params.output_format = FAAC_STREAM_ADTS;
  params.bit_rate = cfg->audio.mic_bitrate_kbps() * 1000;
  params.bandwidth = 0;
  params.use_tns = false;
  params.joint_mode = FAAC_JOINT_NONE;

  st = faac_encoder_open(&params, &handle);
  if (st != FAAC_OK) {
    LOG_ERROR("faac_encoder_open failed: " << faac_strerror(st));
    handle = nullptr;
    return -1;
  }

  faac_encoder_info info;
  info.struct_size = sizeof(info);
  st = faac_encoder_get_info(handle, &info);
  if (st != FAAC_OK) {
    LOG_ERROR("faac_encoder_get_info failed: " << faac_strerror(st));
    faac_encoder_close(&handle);
    return -1;
  }

  inputSamples = info.frame_samples;
  maxOutputBytes = info.max_output_bytes;
  lastFramePtsUs = 0;
  ptsAnchorUs = 0;
  outSamples = 0;
  outTsRunning = false;

  // FIFO: capacity = 3x frame size so we can accumulate across HAL frames
  int fifoCapacity = (int)inputSamples * numChn * 3;
  fifo = std::make_unique<SampleFifo>(numChn);
  fifo->setCapacity(fifoCapacity);

  // Heap-allocated output buffer for FAAC (avoids stack overflow on MIPS)
  delete[] encOutBuf;
  encOutBuf = new uint8_t[maxOutputBytes + 4096];

  // Capture the real AudioSpecificConfig from FAAC
  {
    const uint8_t *ascPtr = nullptr;
    uint32_t aLen = 0;
    if (faac_encoder_asc(handle, &ascPtr, &aLen) == FAAC_OK &&
        ascPtr && aLen > 0 && aLen <= sizeof(asc)) {
      memcpy(asc, ascPtr, aLen);
      ascLen = aLen;
    }
  }

  LOG_INFO("FAAC encoder: " << sampleRate << "Hz"
           << " maxOutputBytes=" << maxOutputBytes
           << " inputSamples=" << inputSamples);

  return 0;
}

int AACEncoder::close() {
  if (handle) {
    faac_encoder_close(&handle);
  }
  handle = nullptr;
  delete[] encOutBuf;
  encOutBuf = nullptr;
  return 0;
}

int AACEncoder::encode(IMPAudioFrame *data, unsigned char *outbuf,
                       int *outLen) {
  if (!handle) {
    LOG_ERROR("FAAC encoder not available");
    return -1;
  }

  const auto frameSamples = (data->len / sizeof(int16_t)) / numChn;
  *outLen = 0;

  fifo->push(reinterpret_cast<const int16_t *>(data->virAddr),
            (int)frameSamples * numChn);

  if (!outTsRunning) {
    if (data->timeStamp != 0) {
      ptsAnchorUs = (int64_t)data->timeStamp;
    } else {
      // HAL timestamps unavailable (e.g. T10) --- use monotonic wall clock
      struct timespec mono;
      clock_gettime(CLOCK_MONOTONIC, &mono);
      ptsAnchorUs = (int64_t)mono.tv_sec * 1000000LL + mono.tv_nsec / 1000;
    }
    outSamples = 0;
    outTsRunning = true;
  }

  const int64_t frameUs =
      (int64_t)inputSamples * 1000000LL / (int64_t)sampleRate;
  int frameTotal = (int)inputSamples * numChn;

  // Drain the FIFO in exact frame-sized chunks
  int16_t frameBuf[2048]; // max HE-AAC: 2048 * numChn (numChn <= 2)
  while (fifo->available() >= frameTotal) {
    fifo->read(frameBuf, frameTotal);

    uint32_t bytesWritten = 0;
    uint32_t outCap = (uint32_t)(maxOutputBytes + 4096);
    faac_status st = faac_encoder_encode(
        handle, frameBuf, (uint32_t)frameTotal,
        encOutBuf, outCap, &bytesWritten);

    if (st != FAAC_OK) {
      LOG_ERROR("FAAC encoding failed: " << faac_strerror(st));
      close();
      if (open() == 0)
        LOG_INFO("AAC encoder reinitialized");
      return -1;
    }

    if (bytesWritten > 0) {
      memcpy(outbuf + *outLen, encOutBuf, bytesWritten);
      *outLen += static_cast<int>(bytesWritten);

      lastFramePtsUs = ptsUsForSamples(outSamples);
      outSamples += (uint64_t)inputSamples;

      // Re-anchor only on a large capture-clock discontinuity: an IMP driver
      // timestamp-domain change (0 -> real time) or a long capture stall.
      //
      // The two values compared here do not describe the same sample boundary.
      // data->timeStamp belongs to the HAL frame just pushed, while
      // ptsUsForSamples(outSamples) is the position of the next AAC frame to be
      // emitted, and the FIFO bridges two different frame sizes -- IMPAudio asks
      // the HAL for the 10 ms multiple closest to 1024 samples (960 at 48 kHz and
      // at 16 kHz, 882 at 44.1 kHz, 1040 at 8 kHz), so the two boundaries drift
      // in and out of phase continuously.  That is normal and must not be
      // mistaken for clock drift.
      //
      // The resulting steady-state offset is bounded by roughly one HAL frame,
      // which by the choice above is close to one AAC frame, so the 4-frame
      // threshold keeps a healthy margin at every supported rate.  Simulated
      // over 4000 frames per rate: worst case 22050 Hz reaches 50.0 ms against a
      // 185.8 ms threshold (3.7x), 48 kHz reaches 20.0 ms against 85.3 ms (4.3x),
      // and no rate triggers a spurious re-anchor.
      //
      // Note there is no per-frame correction here.  The previous code nudged the
      // timeline by +-1 ms on every frame to chase the HAL clock, which is what
      // destroyed the AAC frame cadence; a sample-derived timeline has no
      // per-frame error to chase.
      if (data->timeStamp != 0) {
        int64_t err = (int64_t)data->timeStamp - ptsUsForSamples(outSamples);
        if (err > frameUs * 4 || err < -frameUs * 4) {
          // Resetting the counter re-bases the timeline on the HAL clock.  The
          // next AAC frame starts from PCM already in the FIFO, so its true
          // position is up to one HAL frame away from data->timeStamp -- a
          // one-off phase error far smaller than the >4-frame discontinuity that
          // got us here, and the timeline is exact again from the next frame on.
          // RtspServer's own guard keeps the RTP timestamp monotonic across the
          // jump, so this does not need to be exact.
          ptsAnchorUs = (int64_t)data->timeStamp;
          outSamples = 0;
        }
      }
    }
  }

  return 0;
}
#endif
