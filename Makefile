install:
	@pip install -r requirements.txt

run:
	@python fly_in.py map.txt

debug:
	@python -m pdb fly_in.py

clean:
	@rm -rf __pycache__ .mypy_cache
	@find . -name "*.pyc" -delete

lint:
	@flake8 .
	@mypy . --warn-return-any --warn-unused-ignores --ignore-missing-imports --disallow-untyped-defs --check-untyped-defs

lint-strict:
	@flake8 .
	@mypy . --strict
