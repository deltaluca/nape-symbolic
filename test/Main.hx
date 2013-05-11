package;

import nape.symbolic.SymbolicConstraint;
import nape.space.Space;
import nape.phys.Body;
import nape.geom.Vec2;
import nape.shape.Circle;
import nape.util.BitmapDebug;

class Main {
    static function main() {
        var lib = flash.Lib.current.stage;
        var space = new Space();
        var debug = new BitmapDebug(lib.stageWidth, lib.stageHeight, 0x333333);
        lib.addChild(debug.display);

        var body = new Body();
        body.position.setxy(lib.stageWidth/2, lib.stageHeight/2);
        body.shapes.add(new Circle(100));
        body.space = space;

        var joint = new SymbolicConstraint("
            body body
            scalar rot

            constraint
                body.rotation - rot
        ");
        joint.setBody("body", body);
        joint.setScalar("rot", 0);
        joint.stiff = false;
        joint.space = space;

        (new haxe.Timer(0)).run = function () {
            joint.setScalar("rot", Math.sin(flash.Lib.getTimer()/1000)*Math.PI*2);
            space.step(1/60);
            debug.clear();
            debug.draw(space);
            debug.flush();
        }
    }
}
