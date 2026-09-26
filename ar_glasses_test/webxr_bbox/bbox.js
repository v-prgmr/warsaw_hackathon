/* Minimal dependency-free WebXR AR wireframe, for Spectacles Browser Lens.
 * This deliberately does not detect objects, use the camera, or send poses.
 */
(() => {
  "use strict";
  const button = document.getElementById("start");
  const status = document.getElementById("status");
  const canvas = document.getElementById("xr-canvas");
  let session = null;
  let referenceSpace = null;
  let gl = null;
  let program = null;
  let boxBuffer = null;
  let boxPlaced = false;
  let lastViewer = null;

  function say(message) { status.textContent = message; }

  function shader(type, source) {
    const value = gl.createShader(type);
    gl.shaderSource(value, source);
    gl.compileShader(value);
    if (!gl.getShaderParameter(value, gl.COMPILE_STATUS)) {
      throw new Error(gl.getShaderInfoLog(value));
    }
    return value;
  }

  function initializeGraphics() {
    program = gl.createProgram();
    gl.attachShader(program, shader(gl.VERTEX_SHADER,
      "attribute vec3 a_position; uniform mat4 u_viewProjection; " +
      "void main() { gl_Position = u_viewProjection * vec4(a_position, 1.0); }"));
    gl.attachShader(program, shader(gl.FRAGMENT_SHADER,
      "precision mediump float; void main() { gl_FragColor = vec4(0.08, 1.0, 0.83, 1.0); }"));
    gl.linkProgram(program);
    if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
      throw new Error(gl.getProgramInfoLog(program));
    }
    boxBuffer = gl.createBuffer();
  }

  // Column-major 4x4 multiplication, matching WebXR/WebGL matrices.
  function multiply(a, b) {
    const out = new Float32Array(16);
    for (let column = 0; column < 4; column++) {
      for (let row = 0; row < 4; row++) {
        out[column * 4 + row] =
          a[row] * b[column * 4] +
          a[4 + row] * b[column * 4 + 1] +
          a[8 + row] * b[column * 4 + 2] +
          a[12 + row] * b[column * 4 + 3];
      }
    }
    return out;
  }

  function rotateByQuaternion(vector, q) {
    // q * vector * inverse(q), optimized for a unit quaternion.
    const cross = [
      q.y * vector[2] - q.z * vector[1],
      q.z * vector[0] - q.x * vector[2],
      q.x * vector[1] - q.y * vector[0]
    ];
    const crossAgain = [
      q.y * cross[2] - q.z * cross[1],
      q.z * cross[0] - q.x * cross[2],
      q.x * cross[1] - q.y * cross[0]
    ];
    return vector.map((value, i) => value + 2 * (q.w * cross[i] + crossAgain[i]));
  }

  function placeBox(transform) {
    const ahead = rotateByQuaternion([0, 0, -1.5], transform.orientation);
    const center = [transform.position.x + ahead[0],
                    transform.position.y + ahead[1],
                    transform.position.z + ahead[2]];
    // 0.6 m wide, 0.8 m high, 0.5 m deep, in WebXR's Y-up local space.
    const corner = (x, y, z) => [center[0] + x * 0.3,
                                center[1] + y * 0.4,
                                center[2] + z * 0.25];
    const c = [corner(-1,-1,-1), corner(1,-1,-1),
               corner(1,1,-1), corner(-1,1,-1),
               corner(-1,-1,1), corner(1,-1,1),
               corner(1,1,1), corner(-1,1,1)];
    const edges = [0,1, 1,2, 2,3, 3,0, 4,5, 5,6, 6,7, 7,4,
                   0,4, 1,5, 2,6, 3,7];
    gl.bindBuffer(gl.ARRAY_BUFFER, boxBuffer);
    gl.bufferData(gl.ARRAY_BUFFER,
      new Float32Array(edges.flatMap(index => c[index])), gl.STATIC_DRAW);
    boxPlaced = true;
    say("AR active. Cyan box placed. Move your head; it should stay in the room.");
  }

  function onFrame(time, frame) {
    if (!session) return;
    session.requestAnimationFrame(onFrame);
    const pose = frame.getViewerPose(referenceSpace);
    if (!pose) return;
    lastViewer = pose.transform;
    if (!boxPlaced) placeBox(lastViewer);

    const layer = session.renderState.baseLayer;
    gl.bindFramebuffer(gl.FRAMEBUFFER, layer.framebuffer);
    gl.clearColor(0, 0, 0, 0);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    gl.useProgram(program);
    gl.bindBuffer(gl.ARRAY_BUFFER, boxBuffer);
    const position = gl.getAttribLocation(program, "a_position");
    gl.enableVertexAttribArray(position);
    gl.vertexAttribPointer(position, 3, gl.FLOAT, false, 0, 0);
    for (const view of pose.views) {
      const viewport = layer.getViewport(view);
      gl.viewport(viewport.x, viewport.y, viewport.width, viewport.height);
      const viewProjection = multiply(view.projectionMatrix,
                                      view.transform.inverse.matrix);
      gl.uniformMatrix4fv(gl.getUniformLocation(program, "u_viewProjection"),
                          false, viewProjection);
      gl.drawArrays(gl.LINES, 0, 24);
    }
  }

  async function start() {
    button.disabled = true;
    try {
      if (!gl) {
        gl = canvas.getContext("webgl", { alpha: true, antialias: true,
                                           xrCompatible: true });
        if (!gl) throw new Error("WebGL is unavailable");
        await gl.makeXRCompatible();
        initializeGraphics();
      }
      session = await navigator.xr.requestSession("immersive-ar", {
        requiredFeatures: ["local"]
      });
      session.addEventListener("end", () => {
        session = null;
        boxPlaced = false;
        button.disabled = false;
        say("AR ended. Select Start AR to try again.");
      });
      // If the browser maps a pinch/tap to XR select, place a new test box.
      session.addEventListener("select", () => {
        if (lastViewer) placeBox(lastViewer);
      });
      referenceSpace = await session.requestReferenceSpace("local");
      session.updateRenderState({ baseLayer: new XRWebGLLayer(session, gl, {
        alpha: true, antialias: true
      }) });
      boxPlaced = false;
      session.requestAnimationFrame(onFrame);
    } catch (error) {
      say("Could not enter AR: " + error.message);
      button.disabled = false;
      if (session) await session.end();
    }
  }

  button.addEventListener("click", start);
  async function checkSupport() {
    if (!window.isSecureContext) {
      say("This page needs HTTPS for WebXR. Open its HTTPS tunnel URL.");
      return;
    }
    if (!navigator.xr) {
      say("This browser has no WebXR API. Open the page in Spectacles Browser Lens.");
      return;
    }
    try {
      if (!await navigator.xr.isSessionSupported("immersive-ar")) {
        say("This Browser Lens reports no immersive-ar support. Check Snap OS / Browser Lens availability.");
        return;
      }
      button.textContent = "Start AR bounding box";
      button.disabled = false;
      say("Ready. Select Start AR while wearing the glasses.");
    } catch (error) {
      say("WebXR check failed: " + error.message);
    }
  }
  checkSupport();
})();
