package zpp_nape.symbolic;

import nape.constraint.UserConstraint;
import nape.geom.Vec2;
import nape.geom.Vec3;
import nape.phys.Body;

import zpp_nape.symbolic.Expr;
import zpp_nape.symbolic.Parser;
import nape.ARRAY;

using zpp_nape.symbolic.Expr.ExprUtils;

class ZPP_SymbolicConstraint {

	public var context:Context = null;

	public var variables:Hash<{type:EType,value:Dynamic}> = null;
	public var bodies:Hash<Body> = null; //map body name to real body
	public var bindices:Array<String> = null; //map body index to body name
	public var limits:Array<{limit:Expr,lower:Expr,upper:Expr}> = null;

	public var posC:Expr = null;
	public var velC:Expr = null;
	public var J:Array<Expr> = null;
	public var effK:Expr = null;
	public var dim:Int = null;
	public var lower:Expr = null;
	public var upper:Expr = null;
	public var scale:Array<Float> = null;
	public var equal:Array<Bool> = null;

    public function new() {
    }

	//takes literal expression and flattens into 2D array of floats.
	public function flatten(e:Expr,vert=true):Array<Array<Float>> {
		return switch(e) {
			case eScalar(x): [[x]];
			case eVector(x,y): [[x],[y]];
			case eRowVector(x,y): [[x,y]];
			case eMatrix(a,b,c,d): [[a,b],[c,d]];
			case eBlock(xs):
				var ys = ExprUtils.map(xs, function(x) return flatten(x,!vert));
				//stack vertically/horizontally
				var out:Array<Array<Float>> = [];
				if(!vert) {
					for(y in 0...ys[0].length) {
						var row = [];
						for(e in ys) row = row.concat(e[y]);
						out.push(row);
					}
				}else {
					for(e in ys) out = out.concat(e);
				}
				out;
			default: null;
		}
	}

	public function dot(a:Array<Array<Float>>,b:ARRAY<Float>) {
		var ret = 0.0;
		for(i in 0...a.length) ret += a[i][0]*b[i]*scale[i];
		return ret;
	}

	public function setvec(e:Expr,vec:ARRAY<Float>,?off:Int=0) {
		switch(e) {
			case eScalar(x): vec[off++] = x;
			case eVector(x,y): vec[off++] = x; vec[off++] = y;
			case eBlock(xs):
				for(x in xs) off = setvec(x,vec,off);
			default: throw "wtf";
		}
		return off;
	}

	public function less(a:Expr,b:Expr) {
		return switch(a) {
			case eScalar(x): switch(b) { case eScalar(y): x < y; default: false; }
			case eVector(x1,y1): switch(b) { case eVector(x2,y2): x1 < x2 || y1 < y2; default: false; }
			default: false;
		}
	}
}
