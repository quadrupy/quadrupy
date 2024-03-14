.PHONY: bindings

clean:
	rm -rf src/quadrupy/bindings/{build,_deps}

bindings:
	cmake -S src/quadrupy/bindings -B build/quadrupy/bindings/build
	cmake --build build/quadrupy/bindings/build -j8