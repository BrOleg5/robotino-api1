# Development image for robotino-api1: native (gcc, clang) and Windows (MinGW-w64 + Wine) toolchains.
# Build from the repository root: docker compose build (or docker build -f docker/Dockerfile .).
FROM ubuntu:24.04

ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# wine64 runs the Windows test binaries cross-compiled with MinGW-w64.
RUN dpkg --add-architecture i386 \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential clang cmake ninja-build git ca-certificates gdb \
        python3 python3-dev python3-venv python3-pip \
        mingw-w64 wine64 wine \
    && rm -rf /var/lib/apt/lists/*

# Non-root user (Ubuntu 24.04 images ship a default "ubuntu" user with UID 1000).
RUN if id -u ubuntu >/dev/null 2>&1; then userdel -r ubuntu; fi \
    && groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME}

USER ${USERNAME}
# Python build tools for the bindings live in a venv, not in the system interpreter.
ENV VIRTUAL_ENV=/home/${USERNAME}/venv
RUN python3 -m venv ${VIRTUAL_ENV} \
    && ${VIRTUAL_ENV}/bin/pip install --no-cache-dir scikit-build-core nanobind pytest numpy
ENV PATH=${VIRTUAL_ENV}/bin:${PATH}
# Wine prefix is created once in the image instead of on every test run.
ENV WINEDEBUG=-all WINEPREFIX=/home/${USERNAME}/.wine
RUN wineboot --init >/dev/null 2>&1 || true

WORKDIR /src
CMD ["bash"]
