import Avatar from 'avatar'
import { Application, Skin } from 'piu/MC'
import Sound from 'piu/Sound'
import MarqueeLabel from 'marquee-label'
import Voices from 'voices'
import Mouse from 'drivers/mouse'

/* global trace */
declare const global: any

interface Voice {
  id: string
  balloon: any
  sound: Sound
}

const voiceList: Voice[] = Object.entries(Voices).map(e => {
  const key = e[0]
  const text = e[1]
  return {
    id: key,
    balloon: new MarqueeLabel({
      state: 0,
      bottom: 10,
      right: 10,
      width: 200,
      height: 40,
      name: 'balloon',
      string: text,
    }),
    sound: new Sound({
      path: `${key}.wav`,
    }),
  }
})
let idx = 0

debugger
const fluid = {
  top: 0,
  right: 0,
  bottom: 0,
  left: 0,
}

const balloon = new MarqueeLabel({
  state: 0,
  bottom: 10,
  right: 10,
  width: 200,
  height: 40,
  name: 'balloon',
  string: 'test',
})

const ap = new Application(null, {
  displayListLength: 2048,
  ...fluid,
  skin: new Skin({ fill: 'white' }),
  contents: [
    new Avatar({
      width: 320,
      height: 240,
      name: 'avatar',
    }),
  ],
})

function startSpeech() {
  if (ap.content('balloon') == null) {
    const v = voiceList[idx++]
    if (idx > voiceList.length) {
      idx = 0
    }
    ap.add(v.balloon)
    ap.content('avatar').delegate('startSpeech')
    v.sound.play(0, 1, () => {
      stopSpeech()
    })
  }
}

function stopSpeech() {
  const balloon = ap.content('balloon')
  if (balloon != null) {
    ap.remove(balloon)
    ap.content('avatar').delegate('stopSpeech')
  }
}

if (global.button != null) {
  global.button.a.onChanged = function() {
    if (this.read()) {
      startSpeech()
    }
  }
}
