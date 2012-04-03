![](http://deltaluca.me.uk/napelogo.jpg)

(NAPE-SYMBOLIC)

Module provides prototyping utility for UserConstraints in Nape, with constraints specified in a small DSL compiled at runtime into a UserConstraint that can be plugged directly into Nape.

# Where to download

Available on github are .hx source-code, with dependencies on nape (either real source, or externs) and Parsex.

From haxelib, you can install nape-symbolic with 'haxelib install nape-symbolic' or when updating, 'haxelib upgrade'.

# Status of module

This module is tested working with positional constraints with limits including block constraints like LineJoint/WeldJoint in Nape. As long as the constraint is well-formed it should work just fine!

# Syntax of DSL

Syntax can be viewed in src/symbolic/Parser.hx, and examples available in /examples dir.

# Extensions needed

Extensions need to be made to allow specification of velocity-only constraints which are not currently possible, as well as crazier things like mixed position/velocity constraints (which would require more work as things like eff-mass matrix would also need to be recomputed in velocity iteratinos etc, and no idea how it would behave!)


