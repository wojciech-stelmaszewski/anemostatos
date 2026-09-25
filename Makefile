# Anemostatos — local workflow. Run `make` for help.

.DEFAULT_GOAL := help
NPM := npm

node_modules: package.json
	$(NPM) install
	@touch node_modules

.PHONY: help install dev test test-watch lint typecheck format check build preview clean

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

clean: ## Remove build output and dependencies
	rm -rf dist node_modules
