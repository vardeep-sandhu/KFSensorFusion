export USER_ID:=$(shell id -u)
export GROUP_ID:=$(shell id -g)

start:
	@echo Build docker image...
	@docker-compose build project

test: check-env
	@echo C++ installed
	@docker-compose run project gcc -v
	@echo python3 installed?
	@docker-compose run project python3 --version
	@echo Numpy installed?
	@docker-compose run project python3 -c "import numpy; print(numpy.version.version)"
	
run: check-env
	@docker-compose run project

clean:
	@echo Removing docker image...
	@docker-compose rm project


check-env:
ifndef DATA
	$(error Please specify where your data is located, export DATA=<path>)
endif