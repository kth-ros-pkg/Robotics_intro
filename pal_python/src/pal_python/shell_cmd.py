#!/usr/bin/env python

import subprocess
import tempfile
import time
import os
import signal

class ShellCmd:

    def __init__(self, cmd, stdin=None, stdout=None, stderr=None):
        if stdin:
            self.inf = stdin
        else:
            self.inf = tempfile.NamedTemporaryFile(mode="r")

        if stdout:
            self.outf = stdout
        else:
            self.outf = tempfile.NamedTemporaryFile(mode="w")

        if stderr:
            self.errf= stderr
        else:
            self.errf = tempfile.NamedTemporaryFile(mode="w")
        self.process = subprocess.Popen(cmd, shell=True, stdin=self.inf,
                                        stdout=self.outf, stderr=self.errf,
                                         preexec_fn=os.setsid)

    def __del__(self):
        if not self.is_done():
            self.kill()
        self.outf.close()
        self.errf.close()
        self.inf.close()

    def get_stdout(self):
        with open(self.outf.name, "r") as f:
            return f.read()

    def get_stderr(self):
        with open(self.errf.name, "r") as f:
            return f.read()

    def get_retcode(self):
        """get retcode or None if still running"""
        return self.process.poll()

    def is_done(self):
        return self.process.poll() != None

    def kill(self):
        os.killpg(self.process.pid, signal.SIGTERM)
        self.process.wait()

    def nice_kill(self, retry_time=2, max_retries=2):
        """
         Attempts to kill with SIGINT, returns if successful
        """
        retries=0
        while (not self.is_done() and retries < max_retries):
            if retries > 0:
                time.sleep(retry_time)
                if self.is_done():
                    break #In case command finished during sleep (ie rosbag)
            os.killpg(self.process.pid, signal.SIGINT)
            retries+=1
        return self.is_done()
