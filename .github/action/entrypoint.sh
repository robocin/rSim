#!/bin/sh -l

echo "Starting entrypoint.sh"

echo "Build wheels for different python versions"
for PYBIN in /opt/python/*/bin; do #3.7,3.8,3.9
    echo "builind dist for ${PYBIN}"
    "${PYBIN}/pip" install pip --upgrade #precisa para buildar
    "${PYBIN}/pip" install scikit-build #precisa para buildar
    "${PYBIN}/python" -m build --wheel --outdir dist/ .
done

echo "auditwheels"
for WHEEL in dist/*; do #3.7,3.8,3.9
    auditwheel repair $WHEEL
done

rm -rf dist/
mv -v wheelhouse/ dist/

echo "setup sdist"
/opt/python/pp37-pypy37_pp73/bin/python -m build --sdist --outdir dist/