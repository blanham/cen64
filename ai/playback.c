#include <AL/al.h>
#include <AL/alc.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, const char *argv[]) {
  int status = EXIT_SUCCESS;
  long int frequency_from_cmdline;
  char *endptr;
  FILE *f;

  ALCdevice *dev;
  ALCcontext *ctx;
  ALuint buffers[2];
  ALuint frequency;
  ALuint source;

  if (argc != 3) {
    printf("Usage: %s <dump> <freq>\n", argv[0]);
    return EXIT_SUCCESS;
  }

  if ((f = fopen(argv[1], "r")) == NULL) {
    fprintf(stderr, "Failed to open '%s'\n", argv[1]);
    return EXIT_FAILURE;
  }

  frequency_from_cmdline = strtol(argv[2], &endptr, 10);

  if (*argv[2] == '\0' || *endptr != '\0' || frequency_from_cmdline <= 0) {
    fprintf(stderr, "Frequency not valid: %s\n", argv[2]);
    return EXIT_FAILURE;
  }

  frequency = frequency_from_cmdline;

  if ((dev = alcOpenDevice(NULL)) == NULL) {
    fprintf(stderr, "Failed to open the OpenAL device.\n");
    return EXIT_FAILURE;
  }

  if ((ctx = alcCreateContext(dev, NULL)) == NULL) {
    fprintf(stderr, "Failed to create an OpenAL context.\n");
    alcCloseDevice(dev);
    return EXIT_FAILURE;
  }

  alcMakeContextCurrent(ctx);

  // Context/device is setup, create some buffers and a source.
  alGenBuffers(sizeof(buffers) / sizeof(*buffers), buffers);
  alGenSources(1, &source);

  if (alGetError() == AL_NO_ERROR) {
    uint32_t buf[1024];
    unsigned i;
    ALint val;

    memset(buf, 0, sizeof(buf));

    for (i = 0; i < sizeof(buffers) / sizeof(*buffers); i++) {
      alBufferData(buffers[i], AL_FORMAT_STEREO16,
        buf, sizeof(buf), frequency);
    }

    alSourceQueueBuffers(source, sizeof(buffers) / sizeof(*buffers), buffers);
    alSourcePlay(source);

    if (alGetError() == AL_NO_ERROR) {
      while (!feof(f)) {
        ALuint buffer;

        // XXX: Check return value...
        fread(buf, sizeof(buf), 1, f);

        if (ferror(f)) {
          fprintf(stderr, "Error while reading source file.\n");
          status = EXIT_FAILURE;
          break;
        }

        do {
          alGetSourcei(source, AL_BUFFERS_PROCESSED, &val);
        } while (val == 0);

        alSourceUnqueueBuffers(source, 1, &buffer);
        alBufferData(buffer, AL_FORMAT_STEREO16, buf, sizeof(buf), frequency);
        alSourceQueueBuffers(source, 1, &buffer);

        alGetSourcei(source, AL_SOURCE_STATE, &val);

        if (val != AL_PLAYING)
          alSourcePlay(source);

        if (alGetError() != AL_NO_ERROR) {
          fprintf(stderr, "Error while playing.\n");
          status = EXIT_FAILURE;
          break;
        }
      }
    }

    else {
      fprintf(stderr, "Failed to play the initial buffer.\n");
      status = EXIT_FAILURE;
    }
  }

  else {
    fprintf(stderr, "Failed to create an OpenAL source.\n");
    status = EXIT_FAILURE;
  }

  alDeleteSources(1, &source);
  alDeleteBuffers(sizeof(buffers) / sizeof(*buffers), buffers);

  alcMakeContextCurrent(NULL);
  alcDestroyContext(ctx);
  alcCloseDevice(dev);

  fclose(f);
  return status;
}

