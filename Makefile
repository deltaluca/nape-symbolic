def:
	haxe -cp src -cp / -lib nape -dce full -main DummySymbolicMain -swf symbolic.swf -swf-version 10 -lib Parsex -D haxe3
	debugfp symbolic.swf

test:
	haxe -lib nape -lib nape-symbolic -dce full -main DummySymbolicMain -swf symbolic.swf -swf-version 10 -lib Parsex -D haxe3
	debugfp symbolic.swf

swc:
	haxe -cp src -cp / -cp ../nape/externs -dce full --macro "include('nape.symbolic')" -swf nape-symbolic.swc -swf-version 10 -lib Parsex

haxelib:
	cd src ; \
	rm -f nape-symboliclib.zip ; \
	zip -r nape-symboliclib . ; \
	haxelib test nape-symboliclib.zip

clean:
	rm symbolic.swf
	rm nape-symbolic.swc
