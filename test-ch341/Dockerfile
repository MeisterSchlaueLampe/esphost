
FROM ghcr.io/esphome/esphome

RUN apt update
RUN apt install -y g++
RUN apt install -y libusb-1.0-0-dev

CMD [ "run", "run.yaml" ]
