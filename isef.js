let vv = p5.Vector
let Air = 1.225;
let Helium = 0.1786;
let g = 9.8;

function dot(a, b) {
  return a.x*b.x + a.y*b.y;
}

function times(a, b) {
  return dot(a[0], b[0]) + dot(a[1], b[1]);
}

function distCon(L, a, b, beta, dt) {
  let d = vv.sub(a.pos, b.pos);
  let C = dot(d, d)-L*L;
  let Vi = [a.vel, b.vel];
  let J = [d, vv.sub(createVector(0,0), d)];
  let Bt = beta/dt * C;
  let M_ef = dot(d, d)/a.m + dot(d, d)/b.m;
  let lambda = -(times(J, Vi)+Bt)/M_ef;
  let F = [vv.mult(J[0], lambda), vv.mult(J[1], lambda)];
  let Vf = [vv.mult(F[0], 1/a.m),
            vv.mult(F[1], 1/b.m)];
  return Vf;
}

function Ball(x, y, m, v) {
    this.pos = createVector(x, y);
    this.vel = createVector(0, 0);
    this.ac = createVector(0, 0);

    this.v0 = v;
    this.d0 = m/v;
    this.C = 0.7;
    this.v = v*this.C;
    this.m = m;

    this.d = function() {return this.m/this.v;}

    this.force = function(f) {this.ac.add(vv.mult(f, 1/this.m));}
    this.update = function(dt) {
        this.vel.add(vv.mult(this.ac, dt));
        this.pos.add(vv.mult(this.vel, dt));
        this.ac = createVector(0, 0);
    }

    this.W = function() {return (this.v0*(Air-this.d0)-this.m)*g;}

    this.pe = 0;
    this.PID = function(target, dt) {
        let kp = 1, kd = 8/30;
        let e = this.pos.y - target;

        let pd = kp * e + (e-this.pe)/dt*kd;
        this.C += pd;
        if (this.C > 1) this.C = 1;
        if (this.C < 0.7) this.C = 0.7;

        this.v = this.C*this.v0;
        this.pe = e;
    }

    this.draw = function() {
        let r = (3*this.v/4/PI)**(1/3);
        circle(this.pos.x*100, this.pos.y*100, r*100*2);
    }
}

let tur = [];
let A, B, V = 5e-3, fov;
function setup() {
    createCanvas(510, 510);

    A = new Ball(width*2.5/600, height/400, V*Helium, V);
    B = new Ball(width*3.5/600, height/400, V*Helium, V);
    frameRate(30);

    for (let i = 0; i < 120; i++) tur.push(noise(i/10)*50);
    fov = PI/3;
}

function draw() {
    background(0);

    let dt = 1/30;
    let off = 0;
    if (frameCount > 100) off = sin(frameCount/30)*20;
    A.PID((height/2+off)/100, dt);
    B.PID((height/2-off)/100, dt);

    // W = critical Weight
    A.force(createVector(0, 0.9*A.W()));
    B.force(createVector(0, 0.9*B.W()));

    // gravity
    A.force(createVector(0, A.m*g));
    B.force(createVector(0, B.m*g));

    // buoyancy
    let Fa = g * (Air-A.d())*A.v;
    A.force(createVector(0, -Fa));
    let Fb = g * (Air-B.d())*B.v;
    B.force(createVector(0, -Fb));

    // updates
    A.update(dt);
    B.update(dt);

    // constrains
    let dif = vv.sub(A.pos, B.pos);
    dif.setMag((0.85-dif.mag())/2);
    A.pos.add(dif);
    B.pos.sub(dif);

    stroke(255);
    strokeWeight(2);
    line(A.pos.x*100, A.pos.y*100, B.pos.x*100, B.pos.y*100);
    let mid = vv.mult(vv.add(A.pos, B.pos), 50);
    circle(mid.x, mid.y, 5);

    A.draw();
    B.draw();

    push();
    noFill();
    stroke(255);
    strokeWeight(2);
    beginShape();
    for (let i = 0, j = 0; j < tur.length; i += (width+5)/tur.length, j++) {
        vertex(i, height-tur[j]-50);
    } endShape();

    stroke('red');
    let normal = vv.sub(B.pos, A.pos);
    normal.mult(100);
    normal = createVector(-normal.y, normal.x);

    beginShape();
    fill(255, 0, 0, 100);
    vertex(mid.x, mid.y);
    for (let i = 0, j = 0; j < tur.length; i += (width+5)/tur.length, j++) {
        let p = createVector(i, height-tur[j]-50);
        if (abs(normal.angleBetween(vv.sub(p, mid))) <= fov/2) vertex(p.x, p.y);
    }
    endShape(CLOSE);
    pop();
}

// Helium density: 0.1786 kg/m^3
// Airs density: 1.225 kg/m^3
// F_B = (ρ_air - ρ_gas) × g × V
