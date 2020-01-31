import { Content, Container, Skin, Texture, Behavior } from 'piu/MC'

const AVATAR_COLOR_FG = 'white'
const AVATAR_COLOR_BG = 'black'
const NAME_LEFTEYE = 'leftEye'
const NAME_RIGHTEYE = 'rightEye'
const NAME_MOUTH = 'mouth'
const NAME_IRIS = 'iris'
const NAME_EYELID = 'eyelid'

function normRand(m: number, s: number): number {
  const a = 1 - Math.random()
  const b = 1 - Math.random()
  const c = Math.sqrt(-2 * Math.log(a))
  if (0.5 - Math.random() > 0) {
    return c * Math.sin(Math.PI * 2 * b) * s + m
  } else {
    return c * Math.cos(Math.PI * 2 * b) * s + m
  }
}

const AvatarIrisSkinTexture = Texture.template({
  path: 'eye-alpha.bmp',
})

const AvatarIrisSkin = Skin.template({
  Texture: AvatarIrisSkinTexture,
  width: 16,
  height: 16,
  variants: 16,
  states: 16,
  color: AVATAR_COLOR_FG,
})

enum Emotion {
  NEUTRAL = 'NEUTRAL',
  ANGRY = 'ANGRY',
  SAD = 'SAD',
  HAPPY = 'HAPPY',
  DOUBTFUL = 'DOUBTFUL',
  COLD = 'COLD',
  HOT = 'HOT',
}
type EyeOpen = number
type MouthOpen = number
type EyebrowOpen = number
type Gaze = {
  x: number
  y: number
}
type BothSides<T> = {
  left: T
  right: T
}
interface FaceContext {
  gaze: Gaze | BothSides<Gaze>
  eyeOpen: EyeOpen | BothSides<EyeOpen>
  eyebrowOpen: EyebrowOpen | BothSides<EyebrowOpen>
  mouthOpen: MouthOpen
  emotion: Emotion
  breath: number
}

interface Intervals {
  gazeInterval: number
  blinkInterval: number
}

const AvatarIris = Content.template(({ top, right, bottom, left, x, y, name }) => ({
  top,
  right,
  bottom,
  left,
  x,
  y,
  name,
  width: 16,
  height: 16,
  skin: new AvatarIrisSkin(),
}))

const AvatarEyelidSkinTexture = Texture.template({
  path: 'eyelid-alpha.bmp',
})

const AvatarEyelidSkin = Skin.template({
  Texture: AvatarEyelidSkinTexture,
  width: 24,
  height: 24,
  variants: 24,
  states: 24,
  color: AVATAR_COLOR_BG,
})

const AvatarEyelid = Content.template(({ top, right, bottom, left, x, y, name }) => ({
  top,
  right,
  bottom,
  left,
  x,
  y,
  name,
  width: 24,
  height: 24,
  interval: 40,
  duration: 40 * 7,
  skin: new AvatarEyelidSkin(),
  Behavior: class extends Behavior {
    onTimeChanged(content: Content) {
      let v = Math.floor(content.fraction * 6)
      content.variant = v
    }
    onFinished(content: Content) {
      content.time = 0
    }
    onUpdate(content: OffsetContainer) {
      const ctx = content.props
    }
  },
}))

const AvatarEye = Container.template(({ top, right, bottom, left, x, y, width, height, name }) => ({
  top,
  right,
  bottom,
  left,
  x,
  y,
  width,
  height,
  name,
  clip: true,
  skin: new Skin({
    fill: AVATAR_COLOR_BG,
  }),
  contents: [
    new AvatarIris({
      top: 4,
      left: 4,
      name: NAME_IRIS,
    }),
    new AvatarEyelid({
      top: 0,
      left: 0,
      name: NAME_EYELID,
    }),
  ],
  Behavior: class extends Behavior {
    onDisplaying(container: OffsetContainer) {
      container.originalPosition = new Map()
      // TODO: make smart
      const iris = container.content(NAME_IRIS)
      if (iris != null) {
        container.originalPosition.set(iris, {
          top: iris.y,
          left: iris.x,
        })
      }
    }
    onBlink(container: Container) {
      container.content(NAME_EYELID).start()
    }
    onGazeChange(container: OffsetContainer, gaze: { x: number; y: number }) {
      const iris = container.content(NAME_IRIS)
      const origPos = container.originalPosition.get(iris)
      if (origPos != null) {
        iris.x = origPos.left + gaze.x * 3
        iris.y = origPos.top + gaze.y * 3
      }
    }
  },
}))

const AvatarMouthSkinTexture = Texture.template({
  path: 'mouth-alpha.bmp',
})

const AvatarMouthSkin = Skin.template({
  Texture: AvatarMouthSkinTexture,
  width: 80,
  height: 40,
  variants: 80,
  states: 40,
  color: AVATAR_COLOR_FG,
})

const AvatarMouth = Content.template(({ top, right, bottom, left, x, y, name }) => ({
  top,
  right,
  bottom,
  left,
  x,
  y,
  name,
  width: 80,
  height: 40,
  duration: 60 * 6,
  interval: 60,
  skin: new AvatarMouthSkin(),
  Behavior: class extends Behavior {
    onTimeChanged(content: Content) {
      let v = Math.floor(content.fraction * 10)
      if (v > 5) {
        v = 10 - v
      }
      content.variant = v
    }
    onFinished(content: Content) {
      content.bubble('onOpenFinished')
      content.time = 0
    }
    onUpdate(content: OffsetContainer) {
      const ctx = content.props
    }
    startSpeech(content: Content) {
      content.loop = true
      content.start()
    }
    stopSpeech(content: Content) {
      content.stop()
      content.loop = false
      content.variant = 0
    }
  },
}))

interface OffsetContainerProps extends Intervals, FaceContext {}
interface OffsetContainer extends Container {
  originalPosition: Map<Content, { top: number; left: number }>
  props: OffsetContainerProps
}

const Avatar = Container.template(({ top, right, bottom, left, x, y, width, height, name }) => ({
  top,
  right,
  bottom,
  left,
  x,
  y,
  width,
  height,
  name,
  skin: new Skin({
    fill: 'black',
  }),
  contents: [
    new AvatarEye({
      left: 78,
      top: 81,
      name: NAME_LEFTEYE,
    }),
    new AvatarEye({
      left: 218,
      top: 84,
      name: NAME_RIGHTEYE,
    }),
    new AvatarMouth({
      left: 120,
      top: 128,
      name: NAME_MOUTH,
    }),
  ],
  interval: 33,
  duration: 330 * 9,
  loop: true,
  Behavior: class extends Behavior {
    onDisplaying(container: OffsetContainer) {
      container.originalPosition = new Map()
      // TODO: make smart
      const leftEye = container.content(NAME_LEFTEYE)
      if (leftEye != null) {
        container.originalPosition.set(leftEye, {
          top: leftEye.y,
          left: leftEye.x,
        })
      }
      const rightEye = container.content(NAME_RIGHTEYE)
      if (rightEye != null) {
        container.originalPosition.set(rightEye, {
          top: rightEye.y,
          left: rightEye.x,
        })
      }
      const mouth = container.content(NAME_MOUTH)
      if (mouth != null) {
        container.originalPosition.set(mouth, {
          top: mouth.y,
          left: mouth.x,
        })
      }
      container.props = {
        gaze: {
          x: 0,
          y: 0,
        },
        breath: 3,
        eyeOpen: 0,
        eyebrowOpen: 0,
        mouthOpen: 0,
        gazeInterval: 4000,
        blinkInterval: 4000,
        emotion: Emotion.NEUTRAL,
      }
      container.start()
    }
    onBleath(container: OffsetContainer, breath: number) {
      const offsetY = 3 * breath
      for (let i = 0; i < 3; i++) {
        const c = container.content(i)
        const origPos = container.originalPosition.get(c)
        if (origPos != null) {
          c.y = origPos.top + offsetY
        }
      }
    }
    startSpeech(container: Container) {
      const mouth = container.content(NAME_MOUTH)
      mouth.delegate('startSpeech')
    }
    stopSpeech(container: Container) {
      const mouth = container.content(NAME_MOUTH)
      mouth.delegate('stopSpeech')
    }
    onTimeChanged(container: OffsetContainer) {
      const f = container.fraction

      // update gaze
      container.props.gazeInterval -= container.interval
      if (container.props.gazeInterval < 0) {
        container.props.gaze = {
          x: Math.random() * 2 - 1,
          y: Math.random() * 2 - 1,
        }
        container.content(NAME_LEFTEYE).delegate('onGazeChange', container.props.gaze)
        container.content(NAME_RIGHTEYE).delegate('onGazeChange', container.props.gaze)
        container.props.gazeInterval = normRand(3000, 3000) + 1000
      }

      // update blink
      container.props.blinkInterval -= container.interval
      if (container.props.blinkInterval < 0) {
        container.content(NAME_LEFTEYE).delegate('onBlink')
        container.content(NAME_RIGHTEYE).delegate('onBlink')
        container.props.blinkInterval = normRand(2000, 2000) + 1000
      }

      // update breath
      const breath = Math.sin(f * 2 * Math.PI)
      this.onBleath(container, breath)
    }
  },
}))

export default Avatar
export { AvatarIris, FaceContext, Emotion, AvatarEye, AvatarMouth }
