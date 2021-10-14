export
SHELL := /bin/bash
PWD ?= pwd_unknown
TORCS_REPO := https://github.com/fmirus/torcs-1.3.7.git
TORCS_SERVER_TAG := 1.3.7

REPO_REV:=$(shell git describe --tags --first-parent --always --dirty)
BASE_IMAGE := torcs_server_base_image
BASE_BUILD_IMAGE := torcs_server_build_image
RUNTIME_BASE_IMAGE_TAG := 18.04
RUNTIME_BASE_IMAGE := $(BASE_IMAGE)
SERVER_RUNTIME_IMAGE_TAG := torcs_server:$(TORCS_SERVER_TAG)

RUNTIME_CLIENT=$(shell docker images --format "{{.Repository}}:{{.Tag}}" torcs_client_* | grep -v -e build_image -e base_image)
CAR_FILE_PATH="../client/CAR"
CAR_TYPE_NAME=$(shell head -n 1 ${CAR_FILE_PATH})
ERROR=0

LABELS += 	--label org.opencontainers.image.created="$(shell date --rfc-3339=seconds)" \
        	--label org.opencontainers.image.source="$(shell git ls-remote --get-url)" \
        	--label org.opencontainers.image.revision="$(shell git log -1 --format=%H)" \
        	--label org.opencontainers.image.version="$(shell git describe --tags --first-parent --always --dirty)"

define nl


endef

ifeq ($(MAKECMDGOALS),simulation)
ifeq ( ,${RUNTIME_TORCS_CLIENT_IMAGE})
$(info \
$(nl)+========================================\
$(nl)| RUNTIME_TORCS_CLIENT_IMAGE was not set.\
$(nl)+========================================\
$(nl)| Check the following:\
$(nl)| $(RUNTIME_CLIENT)\
$(nl)+========================================\
$(nl))
ERROR=1
endif
ifeq ( ,${CAR_NAME})
$(info \
$(nl)+========================================\
$(nl)| CAR_NAME was not set.\
$(nl)+========================================\
$(nl)| Check the following:\
$(nl)| $(CAR_TYPE_NAME)\
$(nl)+========================================\
$(nl))
ERROR=1
endif
ifeq ( ,${ROAD_NAME})
$(info \
$(nl)+========================================\
$(nl)| ROAD_NAME was not set.\
$(nl)+========================================\
$(nl)| Default will be used:\
$(nl)| forza\
$(nl)+========================================\
$(nl))
ROAD_NAME:=forza
endif
ifeq (1, ${ERROR})
$(error Error!!!!)
endif
endif


.PHONY: help clone-torcs-repo base-image base-build-image build-runtime-image build-torcs-server build-server full-build clean simulation

help:
	@echo '------------------------------------'
	@echo '| TORCS Docker Makefile'
	@echo '|'
	@echo '| Source of TORCS: $(TORCS_REPO)'
	@echo '| Docker Runtime Image Tag: $(SERVER_RUNTIME_IMAGE_TAG)'
	@echo '|'
	@echo '------------------------------------'
	@echo '|'
	@echo '| help                Shows this help'
	@echo '| base-image          Creates base images'
	@echo '| build-base-image    Creates build image for TORCS server'
	@echo '| build-runtime-image Creates runtime image for TORCS server'
	@echo '| clone-torcs-repo    Clones TORCS repo to tmp folder'
	@echo '| build-server        Builds runtime server images and cleans temp folder'
	@echo '| full-build          Creates base image, build base image, builds torcs server and builds server runtime image and cleans temp folder'
	@echo '| clean               Clean tmp folder to clone TORCS repo'
	@echo '|'
	@echo '------------------------------------'
	@echo '|'
	@echo '| simulation RUNTIME_TORCS_CLIENT_IMAGE=<image_name> CAR_NAME=<car_name> ROAD_NAME=<road_name>       Starts simulation with parameter for runtime client images car_name or map name'
	@echo '|'
	@echo '------------------------------------'

clone-torcs-repo:
	$(eval TEMPFOLDER:=$(shell mktemp -d))
	@echo ''
	@echo '$@'
	@echo ''
	@echo 'Cloneing of $(TORCS_REPO)'
	@echo ''
	@echo 'To $(TEMPFOLDER)'
	@echo ''
	@echo ''
	git clone $(TORCS_REPO) $(TEMPFOLDER)

base-image:
	@echo ''
	@echo '$@'
	@echo ''
	@echo ''
	docker build \
	--network host \
	-f Dockerfile.base \
	-t $(BASE_IMAGE) \
	.

base-build-image:
	@echo ''
	@echo '$@'
	@echo ''
	@echo ''
	docker build \
	--build-arg base_image=${BASE_IMAGE} \
	--network host \
	-f Dockerfile.build \
	-t $(BASE_BUILD_IMAGE) \
	.

build-torcs-server: clone-torcs-repo
	@echo ''
	@echo '$@'
	@echo ''
	@echo 'PWD: ${PWD}'
	docker run \
	-v $(TEMPFOLDER):/torcs \
	$(BASE_BUILD_IMAGE)

build-runtime-image: build-torcs-server
	@echo ''
	@echo '$@'
	@echo ''
	@echo ''
	$(eval BUILD_DIR:=${TEMPFOLDER}/BUILD)
	@echo "${BUILD_DIR}"
	@echo '${RUNTIME_BASE_IMAGE}'
	@echo ''
	cp entrypoint-runtime.sh $(BUILD_DIR) \
	&& \
	docker build \
		${LABELS} \
		--build-arg torcs_img=${RUNTIME_BASE_IMAGE}\
		--network host \
		-t ${SERVER_RUNTIME_IMAGE_TAG} \
		-f Dockerfile.runtime \
		${BUILD_DIR}

clean:
	@echo ''
	@echo '$@'
	@echo ''
	@echo ''
	@echo '${TEMPFOLDER}'
	rm -rdf ${TEMPFOLDER}

build-server: build-runtime-image

full-build: base-image base-build-image build-server clean

clean-compose:
	@echo ''
	@echo '$@'
	@echo ''
	@echo ''
	docker-compose rm -fv

simulation: clean-compose
	@echo ''
	@echo '$@'
	@echo ''
	@echo ''
	docker-compose up  | tee make.simulation.log

show-runtime-client:
	@echo ''
	@echo '$@'
	@echo ''
	@echo 'The following runtime client images are there:'
	@echo '${RUNTIME_CLIENT}'
