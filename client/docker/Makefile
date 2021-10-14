export
SHELL := /bin/bash

SOURCE_DIR := $(PWD)

BASE_IMAGE := torcs_client_base_image
BASE_BUILD_IMAGE := torcs_client_build_image
RUNTIME_BASE_IMAGE_TAG := 18.04
RUNTIME_BASE_IMAGE := $(BASE_IMAGE)
CLIENT_RUNTIME_IMAGE_TAG := torcs_client_${USER}

LABELS += 	--label org.opencontainers.image.created="$(shell date --rfc-3339=seconds)" \
        	--label org.opencontainers.image.source=${TORCS_CLIENT_REPO} \
        	--label org.opencontainers.image.revision="$(shell git log -1 --format=%H)" \
        	--label org.opencontainers.image.version="$(shell git describe --tags --first-parent --always --dirty)"

.PHONY: help base-image base-build-image build-torcs-client build-runtime-image build-clean build-all

help:
	@echo '------------------------------------'
	@echo '| TORCS Client Docker Makefile'
	@echo '|'
	@echo '| Docker Runtime Image Tag: $(CLIENT_RUNTIME_IMAGE_TAG)'
	@echo '| Source path: ${SOURCE_DIR}'
	@echo '|'
	@echo '------------------------------------'
	@echo '|'
	@echo '| help                Shows this help'
	@echo '| build-all           Builds everything and cleans up'
	@echo '|'
	@echo '------------------------------------'

base-image:
	@echo ''
	@echo '$@'
	@echo ''
	@echo ''
	docker build \
	-f docker/Dockerfile.base \
	--network=host \
	-t $(BASE_IMAGE) \
	.

base-build-image: base-image
	@echo ''
	@echo '$@'
	@echo ''
	@echo ''
	docker build \
	--build-arg base_image=${BASE_IMAGE} \
	-f docker/Dockerfile.build \
	--network=host \
	-t $(BASE_BUILD_IMAGE) \
	.

build-torcs-client: base-build-image
	@echo ''
	@echo '$@'
	@echo ''
	@echo ''
	docker run \
	-v $(SOURCE_DIR):/torcs_client \
	$(BASE_BUILD_IMAGE)

build-runtime-image: build-torcs-client
	@echo ''
	@echo '$@'
	@echo ''
	@echo ''
	@echo 'PWD: ${PWD}'
	docker build \
		${LABELS} \
		--build-arg torcs_img=${RUNTIME_BASE_IMAGE}\
		--network=host \
		-t ${CLIENT_RUNTIME_IMAGE_TAG} \
		-f docker/Dockerfile.runtime \
		${SOURCE_DIR}

build-clean: 
	@make -f Makefile.linux clean

build-all: build-runtime-image build-clean
