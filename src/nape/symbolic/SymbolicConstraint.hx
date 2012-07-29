package nape.symbolic;

import nape.constraint.UserConstraint;
import nape.geom.Vec2;
import nape.geom.Vec3;
import nape.phys.Body;

import zpp_nape.symbolic.Expr;
import zpp_nape.symbolic.Parser;
import zpp_nape.symbolic.SymbolicConstraint;
import nape.ARRAY;

using zpp_nape.symbolic.Expr.ExprUtils;

/**
 * Class provides run-time definition of constraints using a simplified DSL.
 * <br/><br/>
 * These constraints are much easier to write and experiment with than a hand-written
 * UserConstraint as only the constraint itself needs to be defined at a high level
 * using the mathematical DSL.
 * <br/><br/>
 * For a handful of constraints, the performance will be perfectly fine and this class
 * provides a debug() method to print internal information that would be required
 * to more easily translate a given constraint into a hand-written and optimised
 * UserConstraint.
 * <br/><br/>
 * Syntax:
 * The input to this class is a constraint definitino which follows this pseudo-BNF
 * grammar:
 * <pre>
 * # this is a comment
 * # body parameter declarations
 * body name [, name]&#42;
 *
 * # variable decalarations
 * scalar|vector name [= default-literal]? [-> time-derivative-expression]?
 *
 * # constraint/variable limits (optional; default 0's)
 * # The type of min/max expressions must match variable/constraint type
 * # with limits imposed component-wise for vector values.
 * # Limits on variables will be checked during constraint verification at runtime.
 * limit expression|constraint min-expression max-expression
 *
 * # expressions may be composed of:
 * # a basic value:
 * #    literal-scalar # eg: 1.5
 * #    inf  # infinity
 * #    eps  # epsilon value (very near 0).
 *
 * # a vector/matrix/block expression:
 * #    [ e1 e2 ] # this is a vector, expressions must be scalar typed.
 * #    [ e1 e2 ; e3 e4 ] # this is a matrix, expressions must be scalar typed.
 * #    { e1 e2 } # this is a block vector, expressions may be of any type.
 *
 * # a let expression:
 * #    let name = e1 in e2
 *
 * # Composed of operations between expressions:
 * #    e1 -+/&#42; e2
 * #    e1 dot e2 # dot product
 * #    e1 cross e2 # cross product
 * #    e1 outer e2 # outer product
 * #    unit expr # unit of expression (sign of scalar, unit vector)
 * #    | expr | # magnitude of expression (magnitude of scalar, length of vector)
 * #    [ expr ] # perpendicular to a vector
 * #    relative name expr # transformation of given vector expression from local
 * #                       # to relative coordinates given a variable name representing
 * #                       # an angle.
 *
 * # A constraint definition:
 * #    For a 1-dimensional constraint, this expression should be of scalar type.
 * #    And for 2 and greater dimensional constraints, a block-vector of scalar types.
 * constraint expr
 * </pre>
 * Each body parameter will automatically define a set of special variables for that body:
 * <pre>
 * body.position   # position of body (vector)
 * body.velocity   # velocity of body (vector)
 * body.rotation   # rotation of body (scalar)
 * body.angularVel # angular velocity of body (scalar)
 * </pre>
 * As an example, we can re-create Nape's DistanceJoint with:
 * <pre>
 * var symbolicDistanceJoint = new SymbolicConstraint("
 *     body body1, body2
 *     vector anchor1, anchor2
 *     scalar jointMin, jointMax
 *
 *     # declare that jointMin <= jointMax, and jointMin, jointMax >= 0
 *     limit jointMin 0 jointMax
 *
 *     # declare constraint as the distance between transformed anchors
 *     constraint
 *        let r1 = relative body1.rotation anchor1 in
 *        let r2 = relative body2.rotation anchor2 in
 *        | (body2.position + r2) - (body1.position + r1) |
 *
 *     # limit distance to the joint min/max.
 *     limit constraint jointMin jointMax
 * ");
 * </pre>
 * And use it like:
 * <pre>
 * symbolicDistanceJoint.space = space;
 * symbolicDistanceJoint.setBody("body1", somebody);
 * ... etc
 * </pre>
 * For more examples visit the github repo at: https://github.com/deltaluca/nape-symbolic
 * <br/><br/>
 * SymbolicConstraint's make use internally of the UserConstraint API, and so like
 * other UserConstraint's can be made stiff, or soft and elastic with no extra work.
 *
 * @requires nape-symbolic module.
 */
class SymbolicConstraint extends UserConstraint {

    /**
     * @private
     */
    public var zns_inner:ZPP_SymbolicConstraint;

    /**
     * Print internal information about symbolic constraint.
     * <br/><br/>
     * This data may be useful should you wish to reimplement a SymbolicConstraint
     * as a more effecient, hand written and optimised UserConstraint.
     *
     * @return A string containing internal information about constraint.
     */
	public function debug():String {
		var ret = "";
		ret += "# constraint context\n";
		ret += "# ------------------\n";
		ret += zns_inner.context.print_context()+"\n";
		ret += "# limits\n";
		ret += "# ------\n";
		for(l in zns_inner.limits) ret += "limit "+l.limit.print()+" "+l.lower.print()+" "+l.upper.print()+"\n";
		ret += "limit constraint "+(zns_inner.lower==null?"#def":zns_inner.lower.print())+" "+
                                   (zns_inner.upper==null?"#def":zns_inner.upper.print())+"\n";
		ret += "\n";
		ret += "# positional constraint\n";
		ret += "# ---------------------\n";
		ret += zns_inner.posC.print()+"\n";
		ret += "\n";
		ret += "# velocity constraint\n";
		ret += "# -------------------\n";
		ret += zns_inner.velC.print()+"\n";
		ret += "# jacobians\n";
		ret += "# ---------\n";
		for(j in zns_inner.J) ret += j.print()+"\n\n";
		ret += "# effective-mass matrix\n";
		ret += "# ---------------------\n";
		ret += zns_inner.effK.print();
		return ret;
	}

	/**
     * Construct a new SymbolicConstraint
     *
     * @param constraint The constraint definition in the format presented in
     *                   documentation for this class.
     * @return A SymbolicConstraint representing definition.
     */
    public function new(constraint:String) {
        zns_inner = new ZPP_SymbolicConstraint();

		var atoms = ConstraintParser.parse(constraint);
		var context = zns_inner.context = ExprUtils.emptyContext();
		var variables = zns_inner.variables = new Hash<{type:EType,value:Dynamic}>();
		var bodies = zns_inner.bodies = new Hash<Body>();
		var limits = zns_inner.limits = new Array<{limit:Expr,lower:Expr,upper:Expr}>();
		var bindices = zns_inner.bindices = [];

		for(a in atoms) {
		switch(a) {
			case aVariables(vars):
				for(v in vars)
					context.variableContext(v.name, v.type, v.del);
			case aBodies(names):
				for(b in names) {
					bindices.push(b);
					context.variableContext(b+".position",etVector,eVariable(b+".velocity"));
					context.variableContext(b+".velocity",etVector);
					context.variableContext(b+".rotation",etScalar,eVariable(b+".angularVel"));
					context.variableContext(b+".angularVel",etScalar);
					context.variableContext(b+"#imass",etScalar);
					context.variableContext(b+"#iinertia",etScalar);
					bodies.set(b, null);
				}
			default:
		}}

		context.extendContext("inf",eScalar(Math.POSITIVE_INFINITY));
		context.extendContext("eps",eScalar(1e-10));

        var posC = null;
		for(a in atoms) {
		switch(a) {
			case aVariables(vars):
				for(v in vars) {
					var def:Dynamic = null;
					if(v.def==null) {
						switch(v.type) {
						case etScalar: def = 0;
						case etVector: def = Vec2.get();
						default:
						}
					}else {
						switch(v.def.simple(context)) {
						case eScalar(x): def = x;
						case eVector(x,y): def = Vec2.get(x,y);
						default:
						}
					}
					variables.set(v.name, {type:v.type, value:def});
				}
			case aConstraint(expr):
				posC = expr.simple(context);
			case aLimit(expr,lower,upper):
				if(expr!=null) limits.push({
					limit:expr.simple(context),
					lower:lower.simple(context),
					upper:upper.simple(context)
				}) else {
					zns_inner.lower = lower.simple(context);
					zns_inner.upper = upper.simple(context);
				}
			default:
		}}

		posC = zns_inner.posC = posC.simple(context);

		//get dimensino of constraint from type
		var type = posC.etype(context);
		var dim = zns_inner.dim = switch(type) {
			case etScalar: 1;
			case etVector: 2;
			case etRowVector: throw "Error: Constraint should not be a row-vector"; 0;
			case etMatrix: throw "Error: Constraint should not be a matrix"; 0;
			case etBlock(xs):
				var dim = 0;
				for(x in xs) {
					dim += switch(x) {
					case etScalar: 1;
					case etVector: 2;
					default: throw "Error: If constraint is a block, it should be exactly a block vector containing nothing but scalars and column vectors"; 0;
					}
				}
				dim;
		}

		var scale = zns_inner.scale = []; var equal = zns_inner.equal = [];
		for(i in 0...dim) { scale.push(0.0); equal.push(true); }

		super(dim);

		//velocity constraint
		var velC = zns_inner.velC = posC.diff(context);

		//jacobian
		var J = zns_inner.J = [];
		for(b in bindices) {
			J.push(velC.diff(context,b+".velocity",0));
			J.push(velC.diff(context,b+".velocity",1));
			J.push(velC.diff(context,b+".angularVel"));
		}

		//effective mass
		var effK = null;
		for(i in 0...J.length) {
			var b = bindices[Std.int(i/3)];
			var m = eVariable(b + (if((i%3)==2) "#iinertia" else "#imass"));
			var e = eLet("#J",J[i],eMul(m,eOuter(eVariable("#J"),eVariable("#J")))).simple(context);

			if(effK==null) effK = e;
			else effK = eAdd(effK,e);
		}
		zns_inner.effK = effK.simple(context);
	}

    /**
     * Access Body type parameter of constraint.
     *
     * @param name The name of the Body type parameter from constraint definition.
     * @return The current value of this parameter.
     * @throws # If name is not a defined parameter of constraint.
     */
	public function getBody(name:String):Null<Body> {
        if (!zns_inner.bodies.exists(name))
            throw "Error: Body parameter with name '"+name+"' is not recognised.";
        return zns_inner.bodies.get(name);
    }

    /**
     * Set Body type parameter of constraint.
     *
     * @param name The name of the Body type parameter from constraint definition.
     * @return A reference to the input body argument.
     * @throws # If name is not a defined parameter of constraint.
     */
	public function setBody(name:String, body:Null<Body>) {
        if (!zns_inner.bodies.exists(name))
            throw "Error: Body parameter with name '"+name+"' is not recognised.";
		zns_inner.bodies.set(name, __registerBody(getBody(name), body));
		return body;
	}

    /**
     * Access Scalar type parameter of constraint.
     *
     * @param name The name of the Scalar type parameter from constraint definition.
     * @return The current value of this parameter
     * @throws # If name is not a defined parameter of constraint.
     */
	public function getScalar(name:String):Float {
        if (!zns_inner.variables.exists(name))
            throw "Error: Scalar parameter with name '"+name+"' is not recognised.";
        return zns_inner.variables.get(name).value;
    }

    /**
     * Access Vector type parameter of constraint.
     *
     * @param name The name of the Vector type parameter from constraint definition.
     * @return The current value of this parameter
     * @throws # If name is not a defined parameter of constraint.
     */
	public function getVector(name:String):Vec2 {
        if (!zns_inner.variables.exists(name))
            throw "Error: Vector parameter with name '"+name+"' is not recognised.";

        var v = zns_inner.variables.get(name).value;
		if(v==null) v = zns_inner.variables.get(name).value = __bindVec2();
        return v;
    }

    /**
     * Set Scalar type parameter of constraint.
     *
     * @param name The name of the Scalar type parameter from constraint definition.
     * @param value The value to set parameter to.
     * @return The input value argument.
     * @throws # If name is not a defined parameter of constraint.
     */
	public function setScalar(name:String,value:Float) {
        if (!zns_inner.variables.exists(name))
            throw "Error: Scalar parameter with name '"+name+"' is not recognised.";
		zns_inner.variables.get(name).value = value;
		__invalidate();
	}

    /**
     * Set Vector type parameter of constraint.
     *
     * @param name The name of the Vector type parameter from constraint definition.
     * @param value The value to set parameter to.
     * @return The input value argument.
     * @throws # If name is not a defined parameter of constraint.
     * @throws # If value is null or has been disposed of.
     */
	public function setVector(name:String,value:Vec2):Vec2 {
        if (!zns_inner.variables.exists(name))
            throw "Error: Vector parameter with name '"+name+"' is not recognised.";
		var v = getVector(name);
		if(v==null) v = zns_inner.variables.get(name).value = __bindVec2();
		return v.set(value);
	}

    /**
     * @private
     */
	public override function __validate() {
        var context = zns_inner.context;
        var variables = zns_inner.variables;
        //TODO:should check for null bodies here.

		//set variables in context
		for(n in variables.keys()) {
			var v = variables.get(n);
			var expr = switch(v.type) {
				case etScalar: eScalar(v.value);
				case etVector: var xy:Vec2 = v.value; eVector(xy.x,xy.y);
				default: null;
			}
			context.replaceContext(n,expr);
		}

        var bodies = zns_inner.bodies;
		//set body parameters in context that remain fixed
		for(n in bodies.keys()) {
			var b = bodies.get(n);
			context.replaceContext(n+"#imass",   eScalar(b.constraintMass));
			context.replaceContext(n+"#iinertia",eScalar(b.constraintInertia));
		}

		//check limits
		//TODO: Ensure any body-variables in limits used are set in context
        var limits = zns_inner.limits;
		for(l in limits) {
			var limit = l.limit.eval(context);
			var lower = l.lower.eval(context);
			var upper = l.upper.eval(context);
			if(zns_inner.less(limit,lower) || zns_inner.less(upper,limit))
                throw "Error: Limit not satisfied on "+l.limit.print()+"="+
                      limit.print()+" and lower="+lower.print()+" and upper="+upper.print();
		}
	}

    /**
     * @private
     */
	public override function __prepare() {
        var context = zns_inner.context;
        var bodies = zns_inner.bodies;
		//set body parameters remaining fixed in velocity iterations
		for(n in bodies.keys()) {
			var b = bodies.get(n);
			context.replaceContext(n+".position", eVector(b.position.x,b.position.y));
			context.replaceContext(n+".rotation", eScalar(b.rotation));
		}
	}

    /**
     * @private
     */
	public override function __position(err:ARRAY<Float>) {
        var context = zns_inner.context;
        var lower = zns_inner.lower;
        var upper = zns_inner.upper;
        var scale = zns_inner.scale;
        var posC = zns_inner.posC;
        var equal = zns_inner.equal;

		zns_inner.setvec(posC.eval(context), err);

		//inequality bounding.
		var bounds = [];
		var lowerv = if(lower==null) null else zns_inner.flatten(lower.eval(context));
		var upperv = if(upper==null) null else zns_inner.flatten(upper.eval(context));
		for(i in 0...err.length) {
			var low  = if(lowerv ==null) 0.0 else lowerv[i][0];
			var high = if(upperv==null) 0.0 else upperv[i][0];
			if(equal[i] = (low==high)) {
				err[i] -= low;
				scale[i] = 1.0;
			}else if(err[i]<low) {
				err[i] = low - err[i];
				scale[i] = -1.0;
			}else if(err[i]>high) {
				err[i] -= high;
				scale[i] = 1.0;
			}else
				err[i] = scale[i] = 0.0;
		}
	}

    /**
     * @private
     */
	public override function __velocity(err:ARRAY<Float>) {
        var context = zns_inner.context;
        var bodies = zns_inner.bodies;
		//set body parameters changing in velocity iterations
		for(n in bodies.keys()) {
			var b = bodies.get(n);
			context.replaceContext(n+".velocity", eVector(b.constraintVelocity.x,b.constraintVelocity.y));
			context.replaceContext(n+".angularVel", eScalar(b.constraintVelocity.z));
		}

        var velC = zns_inner.velC;
        var scale = zns_inner.scale;
		zns_inner.setvec(velC.eval(context), err);
		for(i in 0...err.length) err[i] *= scale[i];
	}

    /**
     * @private
     */
	public override function __eff_mass(out:ARRAY<Float>) {
        var context = zns_inner.context;
        var effK = zns_inner.effK;
        var scale = zns_inner.scale;

		var eff = effK.eval(context);

		//take eff; and produce a full matrix
		var K = zns_inner.flatten(eff);

		//populate out array based on full eff-mass
		var i = 0;
		for(y in 0...K.length) {
			for(x in y...K.length)
				out[i++] = (scale[x]*scale[y])*K[y][x];
		}
	}

    /**
     * @private
     */
	public override function __clamp(imp:ARRAY<Float>) {
        var scale = zns_inner.scale;
        var equal = zns_inner.equal;

		for(i in 0...imp.length) if(!equal[i] && imp[i]>0 || scale[i]==0) imp[i] = 0;
	}

    /**
     * @private
     */
	public override function __impulse(imp:ARRAY<Float>,body:Body,out:Vec3) {
        var context = zns_inner.context;
        var bodies = zns_inner.bodies;
        var bindices = zns_inner.bindices;
        var J = zns_inner.J;

		var bname = "";
		for(n in bodies.keys()) { if(bodies.get(n)==body) { bname = n; break; } }
		var bind = -1;
		for(i in 0...bindices.length) { if(bindices[i]==bname) { bind = i; break; } }

		//these should all be column vectors like [[j0],[j1]...[j(dim-1)]]
		var Jx = zns_inner.flatten(J[bind*3+0].eval(context));
		var Jy = zns_inner.flatten(J[bind*3+1].eval(context));
		var Jz = zns_inner.flatten(J[bind*3+2].eval(context));

		out.x = zns_inner.dot(Jx, imp);
		out.y = zns_inner.dot(Jy, imp);
		out.z = zns_inner.dot(Jz, imp);
	}
}
