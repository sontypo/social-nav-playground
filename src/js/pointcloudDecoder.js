// Pre-computed Base64 decoding lookup table for ultra-fast native byte parsing
const B64_LUT = new Int8Array(256).fill(-1);
const B64_CHARS = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/';
for (let i = 0; i < 64; i++) {
  B64_LUT[B64_CHARS.charCodeAt(i)] = i;
}

export function base64ToUint8Array(b64Str) {
  const len = b64Str.length;
  if (len === 0) return new Uint8Array(0);
  let placeHolders = 0;
  if (b64Str[len - 1] === '=') placeHolders++;
  if (b64Str[len - 2] === '=') placeHolders++;

  const byteLen = Math.max(0, (len * 3) / 4 - placeHolders);
  const bytes = new Uint8Array(byteLen);

  let byteIdx = 0;
  for (let i = 0; i < len; i += 4) {
    const enc1 = B64_LUT[b64Str.charCodeAt(i)];
    const enc2 = B64_LUT[b64Str.charCodeAt(i + 1)];
    const enc3 = B64_LUT[b64Str.charCodeAt(i + 2)];
    const enc4 = B64_LUT[b64Str.charCodeAt(i + 3)];

    if (enc1 === -1 || enc2 === -1) break;

    bytes[byteIdx++] = (enc1 << 2) | (enc2 >> 4);
    if (enc3 !== -1 && byteIdx < byteLen) {
      bytes[byteIdx++] = ((enc2 & 15) << 4) | (enc3 >> 2);
    }
    if (enc4 !== -1 && byteIdx < byteLen) {
      bytes[byteIdx++] = ((enc3 & 3) << 6) | enc4;
    }
  }

  return bytes;
}

// Pre-computed 256-level Turbo & Intensity Colormap Look-up Tables (Zero string allocations per point)
const TURBO_LUT = new Array(256);
const INTENSITY_LUT = new Array(256);

for (let i = 0; i < 256; i++) {
  const t = i / 255.0;
  let r = 0, g = 0, b = 0;
  if (t < 0.25) {
    const s = t / 0.25;
    r = 30 + Math.round(s * 20);
    g = Math.round(s * 200);
    b = 255;
  } else if (t < 0.5) {
    const s = (t - 0.25) / 0.25;
    r = 50 + Math.round(s * 30);
    g = 200 + Math.round(s * 55);
    b = 255 - Math.round(s * 200);
  } else if (t < 0.75) {
    const s = (t - 0.5) / 0.25;
    r = 80 + Math.round(s * 175);
    g = 255 - Math.round(s * 55);
    b = 55 - Math.round(s * 55);
  } else {
    const s = (t - 0.75) / 0.25;
    r = 255;
    g = 200 - Math.round(s * 170);
    b = Math.round(s * 30);
  }
  TURBO_LUT[i] = `rgb(${r},${g},${b})`;

  const ir = Math.round(40 + t * 215);
  const ig = Math.round(60 + t * 180);
  const ib = Math.round(100 + (1 - t) * 120);
  INTENSITY_LUT[i] = `rgb(${ir},${ig},${ib})`;
}

// Turbo / Jet Colormap Lookup
export function turboColormap(t) {
  const idx = Math.max(0, Math.min(255, (t * 255) | 0));
  return TURBO_LUT[idx] || TURBO_LUT[0];
}

// Intensity / Reflectivity Colormap Lookup
export function intensityColormap(intensity, minI = 0, maxI = 100) {
  const norm = Math.max(0, Math.min(1, (intensity - minI) / (maxI - minI || 1)));
  const idx = (norm * 255) | 0;
  return INTENSITY_LUT[idx] || INTENSITY_LUT[0];
}

/**
 * Decode ROS2 sensor_msgs/msg/PointCloud2 Binary Base64 / Byte Stream
 */
export function decodePointCloud2(msg, maxPoints = 50000) {
  if (!msg) return [];

  // 1. Direct Points Array format (Fallback or JSON stream)
  if (Array.isArray(msg.points)) {
    return msg.points.map(p => ({
      x: typeof p.x === 'number' ? p.x : 0,
      y: typeof p.y === 'number' ? p.y : 0,
      z: typeof p.z === 'number' ? p.z : 0,
      intensity: typeof p.intensity === 'number' ? p.intensity : 1.0
    }));
  }

  if (!msg.data) return [];

  // 2. Identify field byte offsets
  let xOff = -1, yOff = -1, zOff = -1, intensityOff = -1, rgbOff = -1;
  const fields = msg.fields || [];
  for (let i = 0; i < fields.length; i++) {
    const f = fields[i];
    const name = f.name.toLowerCase();
    if (name === 'x') xOff = f.offset;
    else if (name === 'y') yOff = f.offset;
    else if (name === 'z') zOff = f.offset;
    else if (name === 'intensity' || name === 'i' || name === 'reflectivity' || name === 'confidence' || name === 'tag') intensityOff = f.offset;
    else if (name === 'rgb' || name === 'rgba') rgbOff = f.offset;
  }

  if (xOff === -1 || yOff === -1 || zOff === -1) {
    xOff = 0; yOff = 4; zOff = 8;
  }

  // 3. Ultra-fast Binary Decoding
  let buffer;
  try {
    if (typeof msg.data === 'string') {
      const bytes = base64ToUint8Array(msg.data);
      buffer = bytes.buffer;
    } else if (Array.isArray(msg.data)) {
      buffer = new Uint8Array(msg.data).buffer;
    } else if (msg.data.buffer) {
      buffer = msg.data.buffer;
    } else {
      return [];
    }
  } catch (e) {
    return [];
  }

  const pointStep = msg.point_step || 16;
  const totalPoints = Math.floor(buffer.byteLength / pointStep);
  if (totalPoints <= 0) return [];

  const dataView = new DataView(buffer);
  const isLittleEndian = !msg.is_bigendian;
  const points = [];
  const stride = totalPoints > maxPoints ? Math.ceil(totalPoints / maxPoints) : 1;

  for (let i = 0; i < totalPoints; i += stride) {
    const offset = i * pointStep;
    if (offset + 12 > buffer.byteLength) break;

    const x = dataView.getFloat32(offset + xOff, isLittleEndian);
    const y = dataView.getFloat32(offset + yOff, isLittleEndian);
    const z = dataView.getFloat32(offset + zOff, isLittleEndian);

    if (isFinite(x) && isFinite(y) && isFinite(z)) {
      let intensity = 1.0;
      if (intensityOff !== -1 && offset + intensityOff + 4 <= buffer.byteLength) {
        intensity = dataView.getFloat32(offset + intensityOff, isLittleEndian);
      }
      points.push({
        x: x,
        y: y,
        z: z,
        intensity: isFinite(intensity) ? intensity : 1.0
      });
    }
  }

  return points;
}

/**
 * 3D Orbit Camera Class
 * Handles 3D spherical orbit projection (Pitch, Yaw, Zoom, Pan) onto 2D Canvas.
 */
export class OrbitCamera3D {
  constructor() {
    this.targetX = 0.0;
    this.targetY = 0.0;
    this.targetZ = 0.5;

    this.yaw = 0.785;       // ~45 deg
    this.pitch = 0.523;     // ~30 deg
    this.distance = 9.0;    // meters
    this.fov = 1.1;         // radians (~63 deg)

    this.minDistance = 1.0;
    this.maxDistance = 60.0;
    this.minPitch = -Math.PI / 2 + 0.01; // Full vertical range (-89 deg looking up)
    this.maxPitch = Math.PI / 2 - 0.01;  // (+89 deg looking top-down)
  }

  rotate(deltaYaw, deltaPitch) {
    this.yaw = (this.yaw + deltaYaw) % (Math.PI * 2);
    this.pitch = Math.max(this.minPitch, Math.min(this.maxPitch, this.pitch + deltaPitch));
  }

  zoom(factor) {
    this.distance = Math.max(this.minDistance, Math.min(this.maxDistance, this.distance * factor));
  }

  pan(deltaX, deltaY, screenScale = 1.0) {
    const sinY = Math.sin(this.yaw);
    const cosY = Math.cos(this.yaw);
    const sinP = Math.sin(this.pitch);
    const cosP = Math.cos(this.pitch);

    const speed = this.distance * 0.0018;
    const moveX = (sinY * deltaX - cosY * sinP * deltaY) * speed;
    const moveY = (-cosY * deltaX - sinY * sinP * deltaY) * speed;
    const moveZ = (cosP * deltaY) * speed;

    this.targetX += moveX;
    this.targetY += moveY;
    this.targetZ += moveZ;
  }

  setTarget(x, y, z = 0.5) {
    this.targetX = x;
    this.targetY = y;
    this.targetZ = z;
  }

  /**
   * Project 3D World Point (wx, wy, wz) -> Screen Pixel (sx, sy, depth, visible)
   */
  project(wx, wy, wz, screenW, screenH) {
    // 1. World to Camera Translation
    const dx = wx - this.targetX;
    const dy = wy - this.targetY;
    const dz = wz - this.targetZ;

    // 2. Spherical Camera Rotation Matrix
    const cosY = Math.cos(this.yaw);
    const sinY = Math.sin(this.yaw);
    const cosP = Math.cos(this.pitch);
    const sinP = Math.sin(this.pitch);

    // Rotate about Z (Yaw)
    const x1 = dx * cosY + dy * sinY;
    const y1 = -dx * sinY + dy * cosY;
    const z1 = dz;

    // Rotate about X (Pitch) and translate along Z by distance
    // In ROS REP 103: +X is Forward, +Y is Left, +Z is Up.
    // Screen +X is Right (-Y in ROS), Screen +Y is Up (+Z in ROS).
    const camX = -y1;
    const camY = z1 * cosP - x1 * sinP;
    const camZ = this.distance + (x1 * cosP + z1 * sinP);

    if (camZ <= 0.2) {
      return { sx: 0, sy: 0, depth: camZ, visible: false };
    }

    // 3. Perspective Projection onto Viewport
    const f = (screenH / 2.0) / Math.tan(this.fov / 2.0);
    const sx = screenW / 2.0 + (camX / camZ) * f;
    const sy = screenH / 2.0 - (camY / camZ) * f; // Invert Y for screen coords

    return {
      sx,
      sy,
      depth: camZ,
      visible: sx >= -50 && sx <= screenW + 50 && sy >= -50 && sy <= screenH + 50
    };
  }
}
