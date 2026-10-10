import functools
from http.server import ThreadingHTTPServer
from pathlib import Path
import tempfile
import threading
import unittest
import urllib.error
import urllib.request
from tools.serve_motion_gallery import MediaHandler


class QuietHandler(MediaHandler):
    def log_message(self, *args):
        pass


class GalleryRanges(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.folder = tempfile.TemporaryDirectory()
        cls.payload = bytes(range(256))*16
        (Path(cls.folder.name)/'clip.mp4').write_bytes(cls.payload)
        cls.server = ThreadingHTTPServer(('127.0.0.1', 0), functools.partial(QuietHandler, directory=cls.folder.name))
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()
        cls.url = f'http://127.0.0.1:{cls.server.server_port}/clip.mp4'

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join()
        cls.folder.cleanup()

    def test_seek_bytes(self):
        with urllib.request.urlopen(urllib.request.Request(self.url, headers={'Range':'bytes=1024-2047'})) as response:
            self.assertEqual(response.status, 206)
            self.assertEqual(response.headers['Content-Range'], 'bytes 1024-2047/4096')
            self.assertEqual(response.headers['Accept-Ranges'], 'bytes')
            self.assertEqual(response.read(), self.payload[1024:2048])

    def test_suffix_and_full_file(self):
        with urllib.request.urlopen(urllib.request.Request(self.url, headers={'Range':'bytes=-16'})) as response:
            self.assertEqual(response.read(), self.payload[-16:])
        with urllib.request.urlopen(self.url) as response:
            self.assertEqual(response.status, 200)
            self.assertEqual(response.read(), self.payload)

    def test_out_of_bounds(self):
        with self.assertRaises(urllib.error.HTTPError) as result:
            urllib.request.urlopen(urllib.request.Request(self.url, headers={'Range':'bytes=4096-'}))
        self.assertEqual(result.exception.code, 416)
        self.assertEqual(result.exception.headers['Content-Range'], 'bytes */4096')
