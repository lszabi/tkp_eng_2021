export
SHELL := /bin/bash
MAKEFLAGS += --no-print-directory

PWD := $(shell pwd)

USER := $(shell docker/github_user.sh)

TORCS_CLIENT_REPO := $(shell git ls-remote --get-url)

.PHONY: help win win-clean linux linux-clean docker

help:
	@echo '------------------------------------'
	@echo '| TORCS Client Makefile'
	@echo '|'
	@echo '| Source of TORCS Client: $(TORCS_CLIENT_REPO)'
	@echo '| Team Name: ${USER}'
	@echo '|'
	@echo '------------------------------------'
	@echo '|'
	@echo '| help       	Shows this help'
	@echo '| win        	Builds Windows Client'
	@echo '| win-clean      Clean Builds Windows Client'
	@echo '| linux      	Builds Linux Client'
	@echo '| linux-clean    Clean Builds Linux Client'
	@echo '| docker     	Builds Linux Client in docker container'
	@echo '------------------------------------'
	@make -f docker/Makefile help

win:
	@echo ''
	@echo '$@'
	@echo ''
	@echo ''
	make -f Makefile.win > make.win.log | type make.win.log

win-clean:
	@echo ''
	@echo '$@'
	@echo ''
	@echo ''
	make -f Makefile.win > make.win.log | type make.win.log

linux:
	@echo ''
	@echo '$@'
	@echo ''
	@echo ''
	make -f Makefile.linux 2>&1 | tee make.linux.log

linux-clean:
	@echo ''
	@echo '$@'
	@echo ''
	@echo ''
	make -f Makefile.linux  clean 2>&1 | tee make.linux.log

docker:
	@echo ''
	@echo '$@'
	@echo ''
	@echo ''
	make -f docker/Makefile build-all 2>&1 | tee make.docker.log
