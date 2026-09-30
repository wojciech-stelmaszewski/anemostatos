# Anemostatos — local workflow. Run `make` for help.

.DEFAULT_GOAL := help
NPM := npm

node_modules: package.json
	$(NPM) install
	@touch node_modules

.PHONY: help install dev test test-watch lint typecheck format check build preview clean train bench book book-data book-figures

help: ## Show this help
	@grep -E '^[a-z-]+:.*## ' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":.*## "}; {printf "  \033[36m%-11s\033[0m %s\n", $$1, $$2}'

install: node_modules ## Install dependencies

dev: node_modules ## Run the simulator (http://localhost:5173)
	$(NPM) run dev -- --open

test: node_modules ## Run unit tests
	$(NPM) test

test-watch: node_modules ## Run unit tests in watch mode
	$(NPM) run test:watch

lint: node_modules ## Lint the code
	$(NPM) run lint

typecheck: node_modules ## Type-check the code
	$(NPM) run typecheck

format: node_modules ## Format the code with Prettier
	$(NPM) run format

check: lint typecheck test ## Lint + typecheck + test (run before committing)

build: node_modules ## Production build into dist/
	$(NPM) run build

preview: build ## Serve the production build locally
	$(NPM) run preview -- --open

NODE_TS := node --experimental-transform-types --no-warnings --import ./scripts/register.mjs

train: node_modules ## Train the neural-network policy (lesson II.20), a few minutes
	$(NODE_TS) scripts/train-policy.ts

bench: node_modules ## Run the controller arena and print the results table
	$(NODE_TS) scripts/bench.ts

LATEX := lualatex -interaction=nonstopmode -halt-on-error -output-directory=build

book: ## Build the book into book/standing-in-the-wind-preview.pdf (needs LuaLaTeX)
	mkdir -p book/build
	cd book && $(LATEX) main.tex > build/latex.out && $(LATEX) main.tex > build/latex.out || (tail -30 build/latex.out; exit 1)
	cp book/build/main.pdf book/standing-in-the-wind-preview.pdf
	@grep -c "undefined" book/build/main.log | xargs -I{} echo "book: {} lines mention undefined references (see book/build/main.log)"

book-data: node_modules ## Fly every experiment of the book and record its telemetry (a few minutes)
	$(NODE_TS) scripts/book-data.ts

book-figures: ## Draw the book's figures from the recorded data (needs Python with matplotlib and pandas)
	python3 book/tools/make_figures.py

clean: ## Remove build output and dependencies
	rm -rf dist node_modules book/build
