# Container image that runs your code
FROM quay.io/pypa/manylinux2010_x86_64

# Install ODE tag 0.16.2 with libccd
RUN git clone --depth 1 --branch 0.16.2 https://bitbucket.org/odedevs/ode.git \
    && export LDFLAGS=-lrt \
    && mkdir ode-build \
    && cd ode-build \
    && cmake ../ode -DCMAKE_BUILD_TYPE=Release -DODE_WITH_DEMOS=OFF -DODE_WITH_LIBCCD=ON \
    && make \
    && make install

# Copies your code file from your action repository to the filesystem path `/` of the container
COPY entrypoint.sh /entrypoint.sh

# Code file to execute when the docker container starts up (`entrypoint.sh`)
ENTRYPOINT ["/entrypoint.sh"]