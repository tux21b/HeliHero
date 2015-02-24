package main

import (
	"errors"
	"fmt"
	"github.com/go-gl/gl/v3.3-core/gl"
	"github.com/go-gl/glfw/v3.1/glfw"
	"github.com/go-gl/mathgl/mgl32"
	"github.com/nfnt/resize"
	"image"
	"image/draw"
	_ "image/jpeg"
	"log"
	"math"
	"os"
	"runtime"
	"strings"
	"unsafe"
)

func init() {
	runtime.LockOSThread()
}

func glDebugCallback(
	source uint32,
	gltype uint32,
	id uint32,
	severity uint32,
	length int32,
	message string,
	userParam unsafe.Pointer) {
	fmt.Printf("Debug source=%d type=%d severity=%d: %s\n", source, gltype, severity, message)
}

type Mesh struct {
	vao     uint32
	vboVert uint32
	vboNorm uint32
	group   *Group
}

func LoadObj(program uint32, path, object string) *Mesh {
	objects, err := Read(path)
	if err != nil {
		log.Fatal(err)
	}
	mesh := &Mesh{group: objects[object].Groups[0]}

	gl.GenVertexArrays(1, &mesh.vao)
	gl.BindVertexArray(mesh.vao)

	gl.GenBuffers(1, &mesh.vboVert)
	gl.BindBuffer(gl.ARRAY_BUFFER, mesh.vboVert)
	gl.BufferData(gl.ARRAY_BUFFER, len(mesh.group.Vertexes)*4,
		gl.Ptr(mesh.group.Vertexes), gl.STATIC_DRAW)

	vertAttrib := uint32(gl.GetAttribLocation(program, gl.Str("vertex_position\x00")))
	gl.EnableVertexAttribArray(vertAttrib)
	gl.VertexAttribPointer(vertAttrib, 3, gl.FLOAT, false, 3*4, gl.PtrOffset(0))

	gl.GenBuffers(1, &mesh.vboNorm)
	gl.BindBuffer(gl.ARRAY_BUFFER, mesh.vboNorm)
	gl.BufferData(gl.ARRAY_BUFFER, len(mesh.group.Normals)*4,
		gl.Ptr(mesh.group.Normals), gl.STATIC_DRAW)
	normAttrib := uint32(gl.GetAttribLocation(program, gl.Str("vertex_normal\x00")))
	gl.EnableVertexAttribArray(normAttrib)
	gl.VertexAttribPointer(normAttrib, 3, gl.FLOAT, false, 3*4, gl.PtrOffset(0))

	return mesh
}

func appendForce(lines *[]float32, pos, dir mgl32.Vec3) {
	a := pos
	b := a.Add(dir)
	*lines = append(*lines, []float32{a[0], a[1], a[2], b[0], b[1], b[2]}...)
}

type Game struct {
	program      uint32
	programSolid uint32
}

func main() {
	log.Println("GLFW version:", glfw.GetVersionString())

	if err := gl.Init(); err != nil {
		log.Fatal(err)
	}

	err := glfw.Init()
	if err != nil {
		log.Fatal(err)
	}
	defer glfw.Terminate()

	glfw.WindowHint(glfw.ContextVersionMajor, 3)
	glfw.WindowHint(glfw.ContextVersionMinor, 2)
	glfw.WindowHint(glfw.OpenGLForwardCompatible, glfw.True)
	glfw.WindowHint(glfw.OpenGLDebugContext, glfw.True)
	glfw.WindowHint(glfw.OpenGLProfile, glfw.OpenGLCoreProfile)
	glfw.WindowHint(glfw.Samples, 4)

	window, err := glfw.CreateWindow(800, 600, "Heli Hero 0.1", nil, nil)
	if err != nil {
		log.Fatal(err)
	}
	defer window.Destroy()

	window.MakeContextCurrent()

	fmt.Println(gl.GoStr(gl.GetString(gl.RENDERER)))
	fmt.Println(gl.GoStr(gl.GetString(gl.VERSION)))

	width, height := window.GetFramebufferSize()

	// Query the extensions to determine if we can enable the debug callback
	var numExtensions int32
	gl.GetIntegerv(gl.NUM_EXTENSIONS, &numExtensions)
	for i := int32(0); i < numExtensions; i++ {
		extension := gl.GoStr(gl.GetStringi(gl.EXTENSIONS, uint32(i)))
		fmt.Println(extension)
		if extension == "GL_ARB_debug_output" {
			gl.Enable(gl.DEBUG_OUTPUT_SYNCHRONOUS_ARB)
			gl.DebugMessageCallbackARB(gl.DebugProc(glDebugCallback), gl.Ptr(nil))
		}
	}

	program, err := newProgram(vertexShader, fragmentShader)
	if err != nil {
		log.Fatal(err)
	}
	gl.UseProgram(program)

	solidProgram, err := newProgram(vertexShaderSolid, fragmentShaderSolid)
	if err != nil {
		log.Fatal(err)
	}

	cubeProgram, err := newProgram(vertexShaderCube, fragmentShaderCube)
	if err != nil {
		log.Fatal(err)
	}

	window.SetSizeCallback(func(w *glfw.Window, newW int, newH int) {
		width = newW
		height = newH
		gl.Viewport(0, 0, int32(newW), int32(newH))
		projection := mgl32.Perspective(70.0, float32(width)/float32(height), 0.1, 10.0)
		gl.UseProgram(program)
		projectionUniform := gl.GetUniformLocation(program, gl.Str("projection\x00"))
		gl.UniformMatrix4fv(projectionUniform, 1, false, &projection[0])
		gl.UseProgram(solidProgram)
		projectionUniformSolid := gl.GetUniformLocation(solidProgram, gl.Str("projection\x00"))
		gl.UniformMatrix4fv(projectionUniformSolid, 1, false, &projection[0])
		gl.UseProgram(cubeProgram)
		projectionUniformCube := gl.GetUniformLocation(cubeProgram, gl.Str("projection\x00"))
		gl.UniformMatrix4fv(projectionUniformCube, 1, false, &projection[0])
	})

	wireframe := false
	window.SetKeyCallback(func(w *glfw.Window, key glfw.Key, scancode int, action glfw.Action, mods glfw.ModifierKey) {
		if key == glfw.KeyEscape && action == glfw.Press {
			w.SetShouldClose(true)
		}
		if key == glfw.KeyQ && action == glfw.Press {
			wireframe = !wireframe
			if wireframe {
				gl.Disable(gl.CULL_FACE)
				gl.PolygonMode(gl.FRONT_AND_BACK, gl.LINE)
			} else {
				gl.Enable(gl.CULL_FACE)
				gl.PolygonMode(gl.FRONT_AND_BACK, gl.FILL)
			}
		}
	})

	heli := LoadObj(program, "heli.obj", "Cube")
	ground := LoadObj(program, "ground.obj", "Ground")

	projection := mgl32.Perspective(mgl32.DegToRad(40), float32(width)/float32(height), 0.01, 100.0)
	camera := mgl32.LookAtV(mgl32.Vec3{3, 1, 0}, mgl32.Vec3{0, 0, 0}, mgl32.Vec3{0, 1, 0})
	model := mgl32.Ident4()

	gl.UseProgram(program)
	projectionUniform := gl.GetUniformLocation(program, gl.Str("projection\x00"))
	cameraUniform := gl.GetUniformLocation(program, gl.Str("view\x00"))
	modelUniform := gl.GetUniformLocation(program, gl.Str("model\x00"))
	gl.UniformMatrix4fv(projectionUniform, 1, false, &projection[0])
	gl.UniformMatrix4fv(cameraUniform, 1, false, &camera[0])
	gl.UniformMatrix4fv(modelUniform, 1, false, &model[0])
	diffuseUniform := gl.GetUniformLocation(program, gl.Str("material_diffuse\x00"))

	gl.UseProgram(solidProgram)
	projectionUniformSolid := gl.GetUniformLocation(solidProgram, gl.Str("projection\x00"))
	cameraUniformSolid := gl.GetUniformLocation(solidProgram, gl.Str("view\x00"))
	modelUniformSolid := gl.GetUniformLocation(solidProgram, gl.Str("model\x00"))
	gl.UniformMatrix4fv(projectionUniformSolid, 1, false, &projection[0])
	gl.UniformMatrix4fv(cameraUniformSolid, 1, false, &camera[0])
	gl.UniformMatrix4fv(modelUniformSolid, 1, false, &model[0])

	gl.UseProgram(cubeProgram)
	projectionUniformCube := gl.GetUniformLocation(cubeProgram, gl.Str("projection\x00"))
	cameraUniformCube := gl.GetUniformLocation(cubeProgram, gl.Str("view\x00"))
	gl.UniformMatrix4fv(projectionUniformCube, 1, false, &projection[0])
	gl.UniformMatrix4fv(cameraUniformCube, 1, false, &camera[0])

	gl.UseProgram(program)

	var lineVAO, lineVBO uint32
	gl.GenVertexArrays(1, &lineVAO)
	gl.BindVertexArray(lineVAO)
	gl.GenBuffers(1, &lineVBO)
	gl.BindBuffer(gl.ARRAY_BUFFER, lineVBO)
	var lines = []float32{0, 0, 0, 0, 1, 0}
	gl.BufferData(gl.ARRAY_BUFFER, len(lines)*4, gl.Ptr(lines), gl.DYNAMIC_DRAW)

	vertAttribLine := uint32(gl.GetAttribLocation(program, gl.Str("vertex_position\x00")))
	gl.EnableVertexAttribArray(vertAttribLine)
	gl.VertexAttribPointer(vertAttribLine, 3, gl.FLOAT, false, 3*4, gl.PtrOffset(0))

	gl.Enable(gl.DEPTH_TEST)
	gl.DepthFunc(gl.LESS)
	gl.ClearColor(0.0, 0.0, 0.0, 1.0)

	gl.Enable(gl.CULL_FACE)
	gl.CullFace(gl.BACK)
	gl.FrontFace(gl.CCW)

	var body RigidBody
	body.invMass = 1.0
	body.linearDamping = .8
	body.angularDamping = .8
	body.invInertiaTensor = mgl32.Ident3()

	cubeTex, cubeVAO, _ := cubeMapInit(cubeProgram)

	previousTime := glfw.GetTime()
	body.Orientation = mgl32.Quat{1.0, mgl32.Vec3{0, 0, 0}}
	for !window.ShouldClose() {
		lines = lines[:0]

		gl.Clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)

		camera := mgl32.LookAtV(mgl32.Vec3{10, 3, 0}, mgl32.Vec3{body.Position[0], body.Position[1], body.Position[2]}, mgl32.Vec3{0, 1, 0})

		gl.DepthMask(false)
		gl.UseProgram(cubeProgram)
		gl.ActiveTexture(gl.TEXTURE0)
		gl.BindTexture(gl.TEXTURE_CUBE_MAP, cubeTex)
		cameraRot := camera.Mul4(mgl32.Translate3D(10, 3, 0))
		gl.UniformMatrix4fv(cameraUniformCube, 1, false, &cameraRot[0])
		gl.BindVertexArray(cubeVAO)
		gl.DrawArrays(gl.TRIANGLES, 0, int32(len(cubeCoords))/3)
		gl.DepthMask(true)

		gl.UseProgram(program)

		time := glfw.GetTime()
		elapsed := time - previousTime
		previousTime = time
		dt := float32(elapsed)

		gl.UseProgram(program)

		body.AddForce(mgl32.Vec3{0.0, -9.81, 0})

		var throttle, yaw, roll, pitch float32
		reset := false
		if glfw.JoystickPresent(glfw.Joystick1) {
			axes := glfw.GetJoystickAxes(glfw.Joystick1)
			throttle = -axes[1]
			yaw = -axes[0]
			pitch = -axes[3]
			roll = axes[2]
			buttons := glfw.GetJoystickButtons(glfw.Joystick1)
			reset = reset || (buttons[0] != 0)
		}

		ruderPos := body.PointToWorld(mgl32.Vec3{0, 0, -2.25})
		ruderDir := body.DirToWorld(mgl32.Vec3{1, 0, 0})
		rotorCenterPos := body.PointToWorld(mgl32.Vec3{0, .5, 0})
		rotorCenterDir := body.DirToWorld(mgl32.Vec3{0, 1, 0})

		if window.GetKey(glfw.KeyW) == glfw.Press {
			throttle += 1
		}
		if window.GetKey(glfw.KeyS) == glfw.Press {
			throttle -= 1
		}
		if window.GetKey(glfw.KeyA) == glfw.Press {
			yaw += 1
		}
		if window.GetKey(glfw.KeyD) == glfw.Press {
			yaw -= 1
		}
		if window.GetKey(glfw.KeyUp) == glfw.Press {
			pitch += 1
		}
		if window.GetKey(glfw.KeyDown) == glfw.Press {
			pitch -= 1
		}
		if window.GetKey(glfw.KeyLeft) == glfw.Press {
			roll -= 1
		}
		if window.GetKey(glfw.KeyRight) == glfw.Press {
			roll += 1
		}

		appendForce(&lines, rotorCenterPos, rotorCenterDir.Mul(throttle))
		body.AddForceAt(rotorCenterDir.Mul(30*throttle), rotorCenterPos)

		appendForce(&lines, ruderPos, ruderDir.Mul(yaw))
		body.AddForceAt(ruderDir.Mul(2*yaw), ruderPos)

		reset = reset || (window.GetKey(glfw.KeyR) == glfw.Press)
		if reset {
			body.Position = mgl32.Vec3{0, 0, 0}
			body.Velocity = mgl32.Vec3{0, 0, 0}
			body.Rotation = mgl32.Vec3{0, 0, 0}
			body.Orientation = mgl32.Quat{1.0, mgl32.Vec3{0, 0, 0}}
		}

		swashplate := (mgl32.Vec3{-roll, 1, pitch}).Normalize()

		numPoints := 8
		strBase := -6.0 / float32(numPoints)
		for i := 0; i < numPoints; i++ {
			angle := float64(i) / float64(numPoints) * 2 * math.Pi
			off := mgl32.Vec3{float32(math.Sin(angle)), 0, float32(math.Cos(angle))}
			str := strBase * swashplate.Dot(off)
			pt := off.Add(mgl32.Vec3{0, 0.5, 0})
			appendForce(&lines, body.PointToWorld(pt), rotorCenterDir.Mul(str))
			body.AddForceAt(rotorCenterDir.Mul(str), body.PointToWorld(pt))
		}
		if body.Position.Y() < 0.6 {
			body.Position[1] = 0.6
			body.Velocity[1] = 0
		}

		model := body.Transform
		gl.UniformMatrix4fv(modelUniform, 1, false, &model[0])

		gl.UniformMatrix4fv(cameraUniform, 1, false, &camera[0])

		gl.Uniform3f(diffuseUniform, 1.0, 0.5, 0.0)
		gl.BindVertexArray(heli.vao)
		gl.DrawArrays(gl.TRIANGLES, 0, int32(len(heli.group.Vertexes)))

		model = mgl32.Ident4()
		gl.UniformMatrix4fv(modelUniform, 1, false, &model[0])
		gl.Uniform3f(diffuseUniform, 0.2, 1.0, 0.0)
		gl.BindVertexArray(ground.vao)
		gl.DrawArrays(gl.TRIANGLES, 0, int32(len(ground.group.Vertexes)))

		if len(lines) > 0 {
			gl.Uniform3f(diffuseUniform, 1.0, 0.0, 0.0)
			gl.UseProgram(solidProgram)
			gl.UniformMatrix4fv(cameraUniformSolid, 1, false, &camera[0])
			gl.BindVertexArray(lineVAO)
			gl.BindBuffer(gl.ARRAY_BUFFER, lineVBO)
			gl.BufferData(gl.ARRAY_BUFFER, len(lines)*4, gl.Ptr(lines), gl.DYNAMIC_DRAW)
			gl.DrawArrays(gl.LINES, 0, int32(len(lines))/3)
		}

		window.SwapBuffers()

		body.Integrate(dt)
		body.ClearForce()

		glfw.PollEvents()
	}
}

func newProgram(vertexShaderSource, fragmentShaderSource string) (uint32, error) {
	vertexShader, err := compileShader(vertexShaderSource, gl.VERTEX_SHADER)
	if err != nil {
		return 0, err
	}

	fragmentShader, err := compileShader(fragmentShaderSource, gl.FRAGMENT_SHADER)
	if err != nil {
		return 0, err
	}

	program := gl.CreateProgram()

	gl.AttachShader(program, vertexShader)
	gl.AttachShader(program, fragmentShader)
	gl.LinkProgram(program)

	var status int32
	gl.GetProgramiv(program, gl.LINK_STATUS, &status)
	if status == gl.FALSE {
		var logLength int32
		gl.GetProgramiv(program, gl.INFO_LOG_LENGTH, &logLength)

		log := strings.Repeat("\x00", int(logLength+1))
		gl.GetProgramInfoLog(program, logLength, nil, gl.Str(log))

		return 0, errors.New(fmt.Sprintf("failed to link program: %v", log))
	}

	gl.ValidateProgram(program)
	gl.GetProgramiv(program, gl.VALIDATE_STATUS, &status)
	if status == gl.FALSE {
		var logLength int32
		gl.GetProgramiv(program, gl.INFO_LOG_LENGTH, &logLength)

		log := strings.Repeat("\x00", int(logLength+1))
		gl.GetProgramInfoLog(program, logLength, nil, gl.Str(log))

		return 0, errors.New(fmt.Sprintf("failed to validate program: %v", log))
	}

	gl.DeleteShader(vertexShader)
	gl.DeleteShader(fragmentShader)

	return program, nil
}

func compileShader(source string, shaderType uint32) (uint32, error) {
	shader := gl.CreateShader(shaderType)

	csource := gl.Str(source)
	gl.ShaderSource(shader, 1, &csource, nil)
	gl.CompileShader(shader)

	var status int32
	gl.GetShaderiv(shader, gl.COMPILE_STATUS, &status)
	if status == gl.FALSE {
		var logLength int32
		gl.GetShaderiv(shader, gl.INFO_LOG_LENGTH, &logLength)

		log := strings.Repeat("\x00", int(logLength+1))
		gl.GetShaderInfoLog(shader, logLength, nil, gl.Str(log))

		return 0, fmt.Errorf("failed to compile %v: %v", source, log)
	}

	return shader, nil
}

var vertexShader string = `
#version 330
uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;
in vec3 vertex_position;
in vec3 vertex_normal;
out vec3 frag_position;
out vec3 frag_normal;

void main() {
    frag_position = vec3(view * model * vec4(vertex_position, 1.0));
    frag_normal = vec3(view * model * vec4(vertex_normal, 0.0));
    gl_Position = projection * vec4(frag_position, 1.0);
}
` + "\x00"

var fragmentShader = `
#version 330

uniform mat4 view;
in vec3 frag_position;
in vec3 frag_normal;

vec3 light_position = vec3(5.0, 3.0, 2.0);
vec3 light_specular = vec3(1.0, 1.0, 1.0);
vec3 light_diffuse = vec3(0.4, 0.4, 0.4);
vec3 light_ambient = vec3(0.02, 0.02, 0.02);

vec3 material_specular = vec3(1.0, 1.0, 1.0);
uniform vec3 material_diffuse = vec3(1.0, 0.0, 0.0);
vec3 material_ambient = vec3(1.0, 1.0, 1.0);
float material_roughness = 20.0;

void main() {
    vec3 ambient = light_ambient * material_ambient;

    vec3 light_position_view = vec3(view * vec4(light_position, 1.0));
    vec3 light_direction = normalize(light_position_view - frag_position);
    float diffuse_factor = max(dot(light_direction, frag_normal), 0.0);
    vec3 diffuse = light_diffuse * material_diffuse * diffuse_factor;

    vec3 half_way = normalize(light_direction - normalize(frag_position));
    float specular_factor = dot(half_way, frag_normal);
    specular_factor = pow(specular_factor, material_roughness);
    vec3 specular = light_specular * material_specular * specular_factor;

    vec3 gamma = vec3(1.0 / 2.2);
    gl_FragColor = vec4(ambient + diffuse + specular, 1.0);
    gl_FragColor.rgb = pow(gl_FragColor.rgb, gamma); 
}
` + "\x00"

var vertexShaderSolid string = `
#version 330
uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;
in vec3 vertex_position;

void main() {
    gl_Position = projection * view * model * vec4(vertex_position, 1.0);
}
` + "\x00"

var fragmentShaderSolid = `
#version 330

void main() {
    gl_FragColor = vec4(1.0, 0.0, 0.0, 1.0);
}
` + "\x00"

var vertexShaderCube string = `
#version 330
uniform mat4 projection;
uniform mat4 view;
in vec3 vertex_position;
out vec3 texcoords;

void main() {
    texcoords = vertex_position;
    gl_Position = projection * view * vec4(vertex_position, 1.0);
}
` + "\x00"

var fragmentShaderCube = `
#version 330

in vec3 texcoords;
uniform samplerCube cube_texture;

void main() {
    gl_FragColor = texture(cube_texture, texcoords);
}
` + "\x00"

var cubeCoords = []float32{
	-1, 1, -1,
	-1, -1, -1,
	1, -1, -1,
	1, -1, -1,
	1, 1, -1,
	-1, 1, -1,

	-1, -1, 1,
	-1, -1, -1,
	-1, 1, -1,
	-1, 1, -1,
	-1, 1, 1,
	-1, -1, 1,

	1, -1, -1,
	1, -1, 1,
	1, 1, 1,
	1, 1, 1,
	1, 1, -1,
	1, -1, -1,

	-1, -1, 1,
	-1, 1, 1,
	1, 1, 1,
	1, 1, 1,
	1, -1, 1,
	-1, -1, 1,

	-1, 1, -1,
	1, 1, -1,
	1, 1, 1,
	1, 1, 1,
	-1, 1, 1,
	-1, 1, -1,

	-1, -1, -1,
	-1, -1, 1,
	1, -1, -1,
	1, -1, -1,
	-1, -1, 1,
	1, -1, 1,
}

func cubeMapInit(program uint32) (uint32, uint32, uint32) {
	var cubetex, vao, vbo uint32
	gl.GenVertexArrays(1, &vao)
	gl.BindVertexArray(vao)
	gl.GenBuffers(1, &vbo)
	gl.BindBuffer(gl.ARRAY_BUFFER, vbo)
	gl.BufferData(gl.ARRAY_BUFFER, len(cubeCoords)*4, gl.Ptr(cubeCoords), gl.STATIC_DRAW)

	vertAttrib := uint32(gl.GetAttribLocation(program, gl.Str("vertex_position\x00")))
	gl.EnableVertexAttribArray(vertAttrib)
	gl.VertexAttribPointer(vertAttrib, 3, gl.FLOAT, false, 3*4, gl.PtrOffset(0))

	gl.ActiveTexture(gl.TEXTURE0)
	gl.GenTextures(1, &cubetex)
	if err := loadCubeSide("posx.jpg", cubetex, gl.TEXTURE_CUBE_MAP_POSITIVE_X); err != nil {
		log.Println(err)
	}
	if err := loadCubeSide("posy.jpg", cubetex, gl.TEXTURE_CUBE_MAP_POSITIVE_Y); err != nil {
		log.Println(err)
	}
	if err := loadCubeSide("posz.jpg", cubetex, gl.TEXTURE_CUBE_MAP_POSITIVE_Z); err != nil {
		log.Println(err)
	}
	if err := loadCubeSide("negx.jpg", cubetex, gl.TEXTURE_CUBE_MAP_NEGATIVE_X); err != nil {
		log.Println(err)
	}
	if err := loadCubeSide("negy.jpg", cubetex, gl.TEXTURE_CUBE_MAP_NEGATIVE_Y); err != nil {
		log.Println(err)
	}
	if err := loadCubeSide("negz.jpg", cubetex, gl.TEXTURE_CUBE_MAP_NEGATIVE_Z); err != nil {
		log.Println(err)
	}
	gl.TexParameteri(gl.TEXTURE_CUBE_MAP, gl.TEXTURE_MAG_FILTER, gl.LINEAR)
	gl.TexParameteri(gl.TEXTURE_CUBE_MAP, gl.TEXTURE_MIN_FILTER, gl.LINEAR)
	gl.TexParameteri(gl.TEXTURE_CUBE_MAP, gl.TEXTURE_WRAP_R, gl.CLAMP_TO_EDGE)
	gl.TexParameteri(gl.TEXTURE_CUBE_MAP, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE)
	gl.TexParameteri(gl.TEXTURE_CUBE_MAP, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE)

	return cubetex, vao, vbo
}

func loadCubeSide(filename string, texture, side uint32) error {
	gl.BindTexture(gl.TEXTURE_CUBE_MAP, texture)
	file, err := os.Open(filename)
	if err != nil {
		return err
	}
	defer file.Close()
	img, _, err := image.Decode(file)
	if err != nil {
		return err
	}

	img = resize.Resize(512, 512, img, resize.Bilinear)
	rgba, ok := img.(*image.RGBA)
	if !ok {
		rgba = image.NewRGBA(image.Rect(0, 0, img.Bounds().Dx(), img.Bounds().Dy()))
		draw.Draw(rgba, rgba.Bounds(), img, img.Bounds().Min, draw.Src)
	}

	gl.TexImage2D(side, 0, gl.RGBA, int32(rgba.Bounds().Dx()),
		int32(rgba.Bounds().Dy()), 0,
		gl.RGBA, gl.UNSIGNED_BYTE, gl.Ptr(rgba.Pix))
	return nil
}
