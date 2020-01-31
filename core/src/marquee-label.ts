import { Container, Label, Skin, Style, Behavior, Texture } from 'piu/MC'
import Timeline from 'piu/Timeline'

const BalloonSkinTexture = Texture.template({
  path: 'balloon.png',
})

const BalloonSkin = Skin.template({
  Texture: BalloonSkinTexture,
  x: 0,
  y: 0,
  width: 80,
  height: 40,
  tiles: { left: 8, right: 8, top: 8, bottom: 8 },
})

const labelStyle = new Style({
  font: 'Cica-Regular',
  color: 'black',
  horizontal: 'left',
  vertical: 'middle',
  left: 4,
})

interface MarqueeContainer extends Container {
  timeline: Timeline
}

const MarqueeLabel = Container.template(({ left, right, top, bottom, width, height, x, y, name, state, string }) => ({
  left,
  right,
  top,
  bottom,
  width,
  height,
  x,
  y,
  name,
  skin: new BalloonSkin(),
  state,
  clip: true,
  Behavior: class extends Behavior {
    startScroll(it: MarqueeContainer) {
      let label = it.first
      if (label == null || !(label instanceof Label)) {
        return
      }
      if (label.width < it.width - 4) {
        return
      }
      let duration = label.string.length * 200
      let timeline
      if (it.timeline == null) {
        timeline = it.timeline = new Timeline()
        timeline.to(
          label,
          {
            x: -label.width,
          },
          duration,
          undefined,
          1000
        )
        it.duration = timeline.duration
      } else {
        timeline = it.timeline
      }
      timeline.seekTo(0)
      it.time = 0
      it.start()
    }
    onDisplaying(it: MarqueeContainer) {
      it.delegate('startScroll')
    }
    onFinished(it: MarqueeContainer) {
      it.bubble('onScrolled')
      it.delegate('startScroll')
    }
    onTimeChanged(it: MarqueeContainer) {
      it.timeline.seekTo(it.time)
    }
  },
  contents: [
    new Label(null, {
      top: 0,
      bottom: 0,
      left: 0,
      style: labelStyle,
      string,
    }),
  ],
}))

export default MarqueeLabel
