#!/bin/sh -l

echo "Starting entrypoint.sh"

# Build wheels for different python versions
# for PYBIN in /opt/python/*/bin; do #3.7,3.8,3.9
#     echo "builind dist for ${PYBIN}"
#     "${PYBIN}/pip" install scikit-build #precisa para buildar
#     "${PYBIN}/python" setup.py bdist_wheel
# done

/opt/python/pp37-pypy37_pp73/bin/pip install scikit-build
/opt/python/pp37-pypy37_pp73/bin/python setup.py bdist_wheel

echo "auditwheels"
for WHEEL in dist/*; do #3.7,3.8,3.9
    auditwheel repair $WHEEL
done

rm -rf dist/
mv -v wheelhouse/ dist/

echo "sdist"
/opt/python/pp37-pypy37_pp73/bin/python setup.py sdist