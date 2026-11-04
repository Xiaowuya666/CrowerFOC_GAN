import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import struct
import tempfile
import unittest
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from motor_music import read_midi, melody_events

def vlq(value):
    out=[value&0x7f];value>>=7
    while value:out.insert(0,(value&0x7f)|0x80);value>>=7
    return bytes(out)

def midi0(events):
    track=b"".join(vlq(dt)+data for dt,data in events)
    return b"MThd"+struct.pack(">IHHH",6,0,1,480)+b"MTrk"+struct.pack(">I",len(track))+track

class MotorMusicTests(unittest.TestCase):
    def test_bundled_midi_melody_can_be_uploaded(self):
        path=Path(__file__).resolve().parents[1]/'music'/'你_从第15小节.mid'
        tracks=read_midi(path)
        self.assertTrue(tracks)
        events=melody_events(tracks[0][1])
        self.assertTrue(events)
        self.assertTrue(all(10<=ms<=60000 for _,ms in events))

    def test_midi_tempo_note_duration_and_pitch(self):
        raw=midi0([(0,b"\xff\x51\x03\x07\xa1\x20"),(0,b"\x90\x45\x64"),(480,b"\x80\x45\x00"),(0,b"\xff\x2f\x00")])
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/"melody.mid";path.write_bytes(raw)
            tracks=read_midi(path)
        self.assertEqual(len(tracks),1)
        notes=melody_events(tracks[0][1])
        self.assertEqual(len(notes),1)
        self.assertEqual(notes[0][0],440)
        self.assertEqual(notes[0][1],500)

    def test_polyphony_selects_highest_pitch_and_keeps_rests(self):
        notes=melody_events([(0,.25,60),(.1,.3,64),(.5,.75,60)])
        self.assertEqual(notes,[(262,100),(330,200),(0,200),(262,250)])

    def test_long_note_splits_without_invalid_short_tail(self):
        chunks=melody_events([(0,60.005,69)])
        self.assertEqual(chunks,[(440,59995),(440,10)])

    def test_short_polyphonic_conversion_fragments_are_folded_without_time_loss(self):
        chunks=melody_events([(0,.2,60),(.095,.104,64)])
        self.assertEqual(chunks,[(262,200)])
        self.assertEqual(sum(ms for _,ms in chunks),200)
        self.assertTrue(all(ms>=10 for _,ms in chunks))

    def test_invalid_short_notes_and_out_of_range_pitches_rejected(self):
        with self.assertRaises(ValueError):melody_events([(0,.005,69)])
        with self.assertRaises(ValueError):melody_events([(0,.02,110)])

if __name__=="__main__":unittest.main()
