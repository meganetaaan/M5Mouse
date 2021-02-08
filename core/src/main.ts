import Avatar from 'avatar'
import { Application, Skin } from 'piu/MC'
import Sound from 'piu/Sound'
import MarqueeLabel from 'marquee-label'
import Voices from 'voices'
import Mouse, { REGISTORY, RegistoryKey } from 'drivers/mouse'
import Timer from 'timer'

/* global trace */
declare const global: any

const commands: RegistoryKey[] = [
  'CALIBRATE',
  'TEST_STRAIGHT',
  'TEST_SPIN',
  'TEST_SLALOM',
  'TEST_ZIG_ZAG',
  'TEST_CRANK',
  'RUN_SEARCH',
  'RUN_FAST'
]
let commandIdx = 0;
let handler: number | null = null;

const mouse = new Mouse()
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
    v.sound.play(0, 1, stopSpeech)
  }
}


function stopSpeech() {
  handler = null
  const balloon = ap.content('balloon')
  if (balloon != null) {
    ap.remove(balloon)
  }
  ap.content('avatar').delegate('stopSpeech')
}

function applyCommand(command: RegistoryKey) {
  const balloon = ap.content('balloon')
  if (balloon != null) {
    ap.remove(balloon)
  }

  let result = 'success'
  try {
    mouse.writeCommand(command)
  } catch(e) {
    result = 'error'
  }
  const b = new MarqueeLabel({
    state: 0,
    bottom: 10,
    right: 10,
    width: 240,
    height: 40,
    name: 'balloon',
    string: `${command}: ${result}`,
  })
  ap.add(b)
  ap.content('avatar').delegate('startSpeech')
  if (handler != null) {
    Timer.clear(handler)
  }
  handler = Timer.set(stopSpeech, 1000)
}

function setCommand(command: RegistoryKey) {
  const balloon = ap.content('balloon')
  if (balloon != null) {
    ap.remove(balloon)
  }

  const b = new MarqueeLabel({
    state: 0,
    bottom: 10,
    right: 10,
    width: 240,
    height: 40,
    name: 'balloon',
    string: `${command}`,
  })
  ap.add(b)
  ap.content('avatar').delegate('startSpeech')
  if (handler != null) {
    Timer.clear(handler)
  }
  handler = Timer.set(stopSpeech, 1000)
}

if (global.button != null) {
  global.button.a.onChanged = function() {
    if (this.read()) {
      commandIdx = Math.max((commandIdx - 1), 0)
      setCommand(commands[commandIdx])
    }
  }
  global.button.b.onChanged = function() {
    if (this.read()) {
      applyCommand(commands[commandIdx])
    }
  }
  global.button.c.onChanged = function() {
    if (this.read()) {
      commandIdx = Math.min((commandIdx + 1), commands.length - 1)
      setCommand(commands[commandIdx])
    }
  }
}
