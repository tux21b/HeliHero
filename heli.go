package main

import (
	"github.com/go-gl/mathgl/mgl32"
	"math"
)

type Vector3 struct {
	X float32
	Y float32
	Z float32
}

func (v *Vector3) Set(x, y, z float32) {
	v.X = x
	v.Y = y
	v.Z = z
}

func (v *Vector3) Add(a, b Vector3) {
	v.X = a.X + b.X
	v.Y = a.Y + b.Y
	v.Z = a.Z + b.Z
}

func (v *Vector3) AddScaled(a, b Vector3, s float32) {
	v.X = a.X + b.X*s
	v.Y = a.Y + b.Y*s
	v.Z = a.Z + b.Z*s
}

func (v *Vector3) Sub(a, b Vector3) {
	v.X = a.X - b.X
	v.Y = a.Y - b.Y
	v.Z = a.Z - b.Z
}

func (v *Vector3) Mul(a Vector3, s float32) {
	v.X = a.X * s
	v.Y = a.Y * s
	v.Z = a.Z * s
}

func (v *Vector3) Len() float32 {
	return float32(math.Sqrt(float64(v.X*v.X + v.Y*v.Y + v.Z*v.Z)))
}

func (v *Vector3) Normalize(a Vector3) {
	l := 1.0 / a.Len()
	v.X = a.X * l
	v.Y = a.Y * l
	v.Z = a.Z * l
}

func (v *Vector3) Cross(a, b Vector3) {
	x, y, z := a.Y*b.Z-a.Z*b.Y, a.Z*b.X-a.X*b.Z, a.X*b.Y-a.Y*b.X
	v.X = x
	v.Y = y
	v.Z = z
}

type Quaternion struct {
	R float32
	I float32
	J float32
	K float32
}

type Matrix3 [9]float32

type Matrix4 [16]float32

type RigidBody struct {
	invMass          float32
	invInertiaTensor mgl32.Mat3
	linearDamping    float32
	angularDamping   float32
	Orientation      mgl32.Quat
	Rotation         mgl32.Vec3

	Position  mgl32.Vec3
	Velocity  mgl32.Vec3
	Transform mgl32.Mat4

	forceAccum  mgl32.Vec3
	torqueAccum mgl32.Vec3
}

func (r *RigidBody) SetDamping(linear, angular float32) {
	r.linearDamping = linear
	r.angularDamping = angular
}

func (r *RigidBody) calculateDerived() {
	r.Transform = mgl32.Translate3D(r.Position[0], r.Position[1], r.Position[2]).Mul4(r.Orientation.Mat4())
}

func (r *RigidBody) Integrate(dt float32) {
	acceleration := r.forceAccum.Mul(r.invMass)
	angularAcceleration := r.invInertiaTensor.Mul3x1(r.torqueAccum)
	r.Velocity = r.Velocity.Add(acceleration.Mul(dt))
	r.Rotation = r.Rotation.Add(angularAcceleration.Mul(dt))
	r.Velocity = r.Velocity.Mul(Pow(r.linearDamping, dt))
	r.Rotation = r.Rotation.Mul(Pow(r.angularDamping, dt))
	r.Position = r.Position.Add(r.Velocity.Mul(dt))
	r.Orientation = r.Orientation.Add((mgl32.Quat{0, r.Rotation.Mul(dt)}).Mul(r.Orientation).Scale(0.5))
	r.Orientation = r.Orientation.Normalize()
	r.calculateDerived()
}

func Pow(a, b float32) float32 {
	return float32(math.Pow(float64(a), float64(b)))
}

func (r *RigidBody) ClearForce() {
	r.forceAccum = mgl32.Vec3{0, 0, 0}
	r.torqueAccum = mgl32.Vec3{0, 0, 0}
}

func (r *RigidBody) AddForce(force mgl32.Vec3) {
	r.forceAccum = r.forceAccum.Add(force)
}

func (r *RigidBody) AddForceAt(force, pos mgl32.Vec3) {
	p := pos.Sub(r.Position)
	r.forceAccum = r.forceAccum.Add(force)
	r.torqueAccum = r.torqueAccum.Add(p.Cross(force))
}

func (r *RigidBody) PointToWorld(pos mgl32.Vec3) mgl32.Vec3 {
	return r.Transform.Mul4x1(mgl32.Vec4{pos[0], pos[1], pos[2], 1.0}).Vec3()
}

func (r *RigidBody) DirToWorld(pos mgl32.Vec3) mgl32.Vec3 {
	return r.Transform.Mul4x1(mgl32.Vec4{pos[0], pos[1], pos[2], 0.0}).Vec3()
}
