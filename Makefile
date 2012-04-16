swc:
	haxe -cp src -cp / -cp ../nape/externs --dead-code-elimination --macro "include('nape.symbolic')" -swf symbolic.swc -swf-version 10 -lib Parsex

haxelib:
	cd src ; \
	rm -f nape-symboliclib.zip ; \
	zip -r nape-symboliclib . ; \
	haxelib test nape-symboliclib.zip
