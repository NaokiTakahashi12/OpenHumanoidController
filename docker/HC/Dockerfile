FROM rbdl:latest
RUN apt update && \
	apt upgrade -y && \
	apt install -y \
		git \
		libboost-thread-dev \
		libboost-system-dev \
		libjpeg-dev \
		libncurses5-dev \
		libeigen3-dev && \
	apt clean && \
	rm -rf /var/lib/apt/lists/*
