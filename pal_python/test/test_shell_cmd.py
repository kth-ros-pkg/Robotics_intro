#!/usr/bin/env python

import unittest
import time
from pal_python import shell_cmd


class TestShellCmd(unittest.TestCase):

    def __init__(self, *args):
        #unittest.TestCase(*args)
        super(TestShellCmd, self).__init__(*args)

    def short_cmd_impl(self, stdin, stdout, stderr):
        cmd = "echo $((2 + 3))"
        shcmd = shell_cmd.ShellCmd(cmd, stdin, stdout, stderr)
        while not shcmd.is_done():
            pass
        self.assertTrue(shcmd.is_done())
        self.assertEqual(shcmd.get_retcode(), 0)
        self.assertEqual(shcmd.get_stdout(), "5\n")
        #Assert out can be retrieved multiple times
        self.assertEqual(shcmd.get_stdout(), "5\n")
        self.assertEqual(shcmd.get_stderr(), "")

    def test_short_cmd(self):
        self.short_cmd_impl(None, None, None)
        with open("/tmp/short_cmd_test_out", "w") as stdout:
            with open("/tmp/short_cmd_test_err", "w") as stderr:
                self.short_cmd_impl(None, stdout, stderr)

    def test_same_file_handle(self):
        with open("/tmp/same_file_handle_test", "w") as stdout:
            cmd = "echo $((2 + 3)) && ls non_existing_file"
            cmd = shell_cmd.ShellCmd(cmd, None, stdout, stdout)
            while not cmd.is_done():
                pass
            self.assertEqual(cmd.get_stdout(), cmd.get_stdout())

    def test_abortable_cmd(self):
        cmd = "yes"
        shcmd = shell_cmd.ShellCmd(cmd)
        self.assertFalse(shcmd.is_done())
        self.assertIsNone(shcmd.get_retcode())
        for i in range(0,4):
            time.sleep(0.2)
            self.assertFalse(shcmd.is_done())
        shcmd.kill()
        self.assertTrue(shcmd.is_done())
        self.assertIsNotNone(shcmd.get_retcode())
        self.assertNotEqual(shcmd.get_retcode(), 0)
        self.assertEqual(shcmd.get_stderr(), "")

    def test_error_cmd(self):
        cmd = "ls /usr && ls non_existing_file"
        shcmd = shell_cmd.ShellCmd(cmd)
        while not shcmd.is_done():
            pass
        self.assertTrue(shcmd.is_done())
        self.assertIsNotNone(shcmd.get_retcode())
        self.assertNotEqual(shcmd.get_retcode(), 0)
        self.assertNotEqual(shcmd.get_stdout(), "")
        self.assertNotEqual(shcmd.get_stderr(), "")

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('pal_python', 'test_shell_cmd', TestShellCmd)

