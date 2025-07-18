venv: .venv/bin/activate

.venv/bin/activate: requirements.txt
	test -d .venv || virtualenv -p python3.6 --prompt="(pylbfgs) " .venv
	. .venv/bin/activate; pip install -r requirements.txt
	touch .venv/bin/activate

clean:
	rm -rf .venv
	rm -rf __pycache__
	rm -rf build
	rm -f *.so

build:
	python setup.py build_ext -i

install:
	python setup.py install
