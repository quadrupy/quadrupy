.PHONY: bindings

clean:
	rm -rf src/quadrupy/bindings/{build,_deps,lib}

bindings:
	cmake -S src/quadrupy/bindings -B src/quadrupy/bindings/build
	cmake --build src/quadrupy/bindings/build -j8