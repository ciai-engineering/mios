FROM ubuntu:22.04

RUN apt-get update
RUN apt-get install -y --no-install-recommends python3-pip
RUN apt-get update && apt-get install -y --no-install-recommends ffmpeg docker.io
RUN pip3 install -Iv 'setuptools==57.5.0'
RUN pip3 install -U numpy
RUN pip3 install -U scipy
RUN pip3 install -U pymongo
RUN pip3 install -U scikit-learn
RUN pip3 install -U requests
RUN pip3 install -U pyDOE3 scikit-optimize
RUN pip3 install -U websockets
RUN pip3 install -U deap
RUN pip3 install -U ifaddr
RUN pip3 install -U redis


COPY ml_service/ /mios_mls/
COPY python/mios_examples.py /mios_mls/mios_examples.py
# ProblemDefinitionFactory resolves the default skill contexts from the
# repository-level Python taxonomy.  Include those files in the standalone
# learning-service image as well.
COPY python/taxonomy/default_contexts/ /python/taxonomy/default_contexts/

EXPOSE 8000 8001

WORKDIR mios_mls

CMD ["python3", "./start_interface.py"]
