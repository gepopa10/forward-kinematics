FROM ubuntu:20.04

# Avoids prompting of timezone in the ubuntu image
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y g++ cmake make git

# Install GoogleTest from the official repo
RUN git clone https://github.com/google/googletest.git && cd /googletest && mkdir build && cd build && cmake .. && make && make install && cd ..

RUN rm -rf /googletest

WORKDIR /app
COPY . /app

# Build the project
RUN rm -rf /googletest build && mkdir build && cd build && cmake .. && make

# This command will be executed when Docker container runs
CMD ["sh", "-c", "cd build && ./main_app && ctest"]

