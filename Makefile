def:
	haxe -cp src -cp / -lib nape --dead-code-elimination -main DummySymbolicMain -swf symbolic.swf -swf-version 10 -lib Parsex
	debugfp symbolic.swf

swc:
	haxe -cp src -cp / -cp $(NAPE_EXTERNS) --dead-code-elimination --macro "include('nape.symbolic')" -swf symbolic.swc -swf-version 10 -lib Parsex

haxelib:
	cd src ; \
	rm -f nape-symboliclib.zip ; \
	zip -r nape-symboliclib . ; \
	haxelib test nape-symboliclib.zip

tar:
	rm -rf nape-symbolic.tar.gz
	tar cvfz nape-symbolic.tar.gz src Makefile version
