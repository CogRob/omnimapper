all: help

help:
	@echo ""
	@echo "-- Help Menu"
	@echo ""
	@echo "   1. make build            - build all cogrob images"
	@echo "   1. make clone            - clone all cogrob images"
	@echo "   1. make clean            - remove all cogrob images"
	@echo "   2. make terminator       - run terminator on omnimapper-dox"
	@echo "   2. make bash             - run bash on omnimapper-dox"
	@echo ""

build:
	@docker build --tag=cogrob/omnimapper-dep dep/.
	@docker build --tag=cogrob/omnimapper-dev dev/.
	@docker build --tag=cogrob/omnimapper-gui gui/.
	@docker build --tag=cogrob/omnimapper-dox dox/.
	@docker build --tag=cogrob/omnimapper-demo demo/.
	# @( cd nvidia ; bash build.sh)
	# @docker build --tag=cogrob/omnimapper-nvidia nvidia/.

clone:
	@docker clone cogrob/omnimapper-dep
	@docker clone cogrob/omnimapper-dev
	@docker clone cogrob/omnimapper-gui
	@docker clone cogrob/omnimapper-dox
	@docker clone cogrob/omnimapper-demo
	# @docker clone cogrob/omnimapper-nvidia
	
clean:
	@docker rmi -f cogrob/omnimapper-dep
	@docker rmi -f cogrob/omnimapper-dev
	@docker rmi -f cogrob/omnimapper-gui
	@docker rmi -f cogrob/omnimapper-dox
	@docker rmi -f cogrob/omnimapper-demo
	# @docker rmi -f cogrob/omnimapper-nvidia

terminator bash:
	@bash run.sh -i -t cogrob/omnimapper-dox $@
