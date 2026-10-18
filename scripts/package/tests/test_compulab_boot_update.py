#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Exercise boot deployment and rollback in a fixture, without real mounts.

Run with: python3 scripts/package/tests/test_compulab_boot_update.py
Requires bash and fakeroot. Device discovery is stubbed; the updater's
deployment, verification, backup and rollback functions run unchanged.
"""

import gzip
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


@unittest.skipUnless(shutil.which("fakeroot"), "fakeroot is required")
class BootUpdateTest(unittest.TestCase):
    release = "6.18.20-test"

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="compulab-boot-test-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        for name in ("boot", "rootboot", "package/dtbs", "work"):
            (self.root / name).mkdir(parents=True)
        self.boot = self.root / "boot"
        self.rootboot = self.root / "rootboot"
        self.package = self.root / "package"
        (self.package / "kernel.gz").write_bytes(gzip.compress(b"new kernel"))
        (self.package / "fit").write_bytes(b"new FIT")
        (self.package / "dtbs/board.dtb").write_bytes(b"new DTB")
        (self.boot / "Image-old").write_bytes(b"old kernel")
        (self.boot / "Image").symlink_to("Image-old")
        (self.boot / "board.dtb").write_bytes(b"old boot DTB")
        (self.boot / "fitImage-old").write_bytes(b"old FIT")
        (self.boot / "fitImage").symlink_to("fitImage-old")
        (self.rootboot / "board.dtb").write_bytes(b"old root DTB")
        package_scripts = Path(__file__).resolve().parents[1]
        self.script = package_scripts / "compulab-boot-update"

    @staticmethod
    def snapshot(directory):
        return {
            path.name: ("link", os.readlink(path)) if path.is_symlink()
            else ("file", path.read_bytes())
            for path in directory.iterdir() if not path.is_dir()
        }

    def run_update(self, with_fit=True, shared=False, overrides="", env=None,
                   fit_path=None):
        source = self.script.read_text().rsplit('\nmain "$@"', 1)[0]
        for old, new in (
            ("readonly BACKUP_ROOT=/boot/kernel-backup.d",
             'readonly BACKUP_ROOT="$TEST_ROOT/backups"'),
            ("readonly WORK_ROOT=/var/tmp",
             'readonly WORK_ROOT="$TEST_ROOT/work"'),
            ("readonly ROOTFS_BOOT_DIR=/boot",
             'readonly ROOTFS_BOOT_DIR="$TEST_ROOT/$TEST_ROOTBOOT"'),
            ("dtb_dir=/usr/lib/linux-image-$release/compulab",
             'dtb_dir="$TEST_ROOT/package/dtbs"'),
            ("/var/tmp/compulab-kernel-postinst.*)",
             '"$TEST_ROOT"/work/compulab-kernel-postinst.*)'),
        ):
            self.assertIn(old, source)
            source = source.replace(old, new)
        source += '''
qualify_root_device() { root_device=; return 0; }
resolve_boot_device() { boot_mount="$TEST_ROOT/boot"; boot_mounted=1; }
sync() { :; }
'''
        source += overrides + '\nmain "$@"\n'
        runner = self.root / "updater.sh"
        runner.write_text(source)
        arguments = [self.release, str(self.package / "kernel.gz")]
        if with_fit:
            arguments.append(str(fit_path or self.package / "fit"))
        environment = os.environ.copy()
        environment.update(TEST_ROOT=str(self.root),
                           TEST_ROOTBOOT="boot" if shared else "rootboot",
                           DEB_MAINT_PARAMS="configure")
        environment.pop("COMPULAB_SKIP_BOOT_UPDATE", None)
        environment.update(env or {})
        return subprocess.run(["fakeroot", "bash", str(runner), *arguments],
                              env=environment, capture_output=True, text=True)

    def test_fit_deploy_and_backup(self):
        result = self.run_update()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(os.readlink(self.boot / "fitImage"),
                         "fitImage-" + self.release)
        self.assertEqual((self.boot / "fitImage").read_bytes(), b"new FIT")
        self.assertEqual((self.boot / "fitImage-old").read_bytes(), b"old FIT")
        self.assertEqual((self.boot / "Image").read_bytes(), b"new kernel")
        backups = list((self.root / "backups").glob("*/files/fitImage"))
        self.assertEqual(len(backups), 1)
        self.assertEqual(os.readlink(backups[0]), "fitImage-old")
        self.assertEqual((backups[0].parent / "fitImage-old").read_bytes(),
                         b"old FIT")

    def test_no_fit_leaves_stale_fit_and_link_untouched(self):
        stale = self.rootboot / ("fitImage-" + self.release)
        stale.write_bytes(b"stale packaged FIT")
        before = self.snapshot(self.boot)
        result = self.run_update(with_fit=False)
        self.assertEqual(result.returncode, 0, result.stderr)
        for name in ("fitImage", "fitImage-old"):
            self.assertEqual(self.snapshot(self.boot)[name], before[name])
        self.assertEqual(stale.read_bytes(), b"stale packaged FIT")
        self.assertFalse((self.boot / ("fitImage-" + self.release)).exists())

    def test_shared_boot_stages_packaged_fit_before_cleanup(self):
        fit = self.boot / ("fitImage-" + self.release)
        fit.write_bytes(b"new FIT on shared boot")
        result = self.run_update(shared=True, fit_path=fit)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual((self.boot / "fitImage").read_bytes(),
                         b"new FIT on shared boot")

    def test_symlink_failure_rolls_back_boot_and_root_dtbs(self):
        boot_before = self.snapshot(self.boot)
        root_before = self.snapshot(self.rootboot)
        result = self.run_update(overrides='''
ln() {
    [[ ${!#} != "$TEST_ROOT/boot/fitImage" ]] || return 1
    command ln "$@"
}
''')
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.snapshot(self.boot), boot_before)
        self.assertEqual(self.snapshot(self.rootboot), root_before)

    def test_failed_initial_deployment_removes_partial_payload(self):
        for path in self.boot.iterdir():
            path.unlink()
        root_before = self.snapshot(self.rootboot)
        result = self.run_update(overrides='''
ln() {
    [[ ${!#} != "$TEST_ROOT/boot/fitImage" ]] || return 1
    command ln "$@"
}
''')
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.snapshot(self.boot), {})
        self.assertEqual(self.snapshot(self.rootboot), root_before)

    def test_failed_verification_restores_existing_versioned_fit(self):
        (self.boot / ("fitImage-" + self.release)).write_bytes(b"previous FIT")
        boot_before = self.snapshot(self.boot)
        root_before = self.snapshot(self.rootboot)
        result = self.run_update(overrides='''
cp() {
    command cp "$@" || return
    if [[ ${!#} == "$TEST_ROOT/boot/fitImage-$release" ]]; then
        printf corrupt > "${!#}"
    fi
}
''')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Installed FIT image failed verification", result.stderr)
        self.assertEqual(self.snapshot(self.boot), boot_before)
        self.assertEqual(self.snapshot(self.rootboot), root_before)

    def test_fit_space_is_checked_before_mutation(self):
        (self.package / "fit").write_bytes(b"F" * (2 * 1024 * 1024))
        boot_before = self.snapshot(self.boot)
        root_before = self.snapshot(self.rootboot)
        result = self.run_update(overrides='''
df() { printf 'Filesystem 1024-blocks Used Available Capacity Mounted on\n';
       printf 'fixture 1024 0 1024 0%% fixture\n'; }
''')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("New boot payload needs", result.stderr)
        self.assertEqual(self.snapshot(self.boot), boot_before)
        self.assertEqual(self.snapshot(self.rootboot), root_before)

    def test_unsupported_root_skips_fit_deployment(self):
        before = self.snapshot(self.boot)
        result = self.run_update(
            overrides="qualify_root_device() { return 2; }")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.snapshot(self.boot), before)
        self.assertEqual((self.rootboot / "board.dtb").read_bytes(), b"new DTB")

    def test_missing_packaged_fit_fails_before_mutation(self):
        before = self.snapshot(self.boot)
        result = self.run_update(fit_path=self.package / "missing-fit")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Packaged FIT image is missing or empty", result.stderr)
        self.assertEqual(self.snapshot(self.boot), before)

    def test_nonfile_fit_destination_fails_before_mutation(self):
        (self.boot / "fitImage").unlink()
        (self.boot / "fitImage").mkdir()
        before = self.snapshot(self.boot)
        result = self.run_update()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("non-file FIT destination", result.stderr)
        self.assertEqual(self.snapshot(self.boot), before)
        self.assertEqual(list((self.boot / "fitImage").iterdir()), [])

    def test_skip_and_nonconfigure_actions(self):
        for env in ({"COMPULAB_SKIP_BOOT_UPDATE": "1"},
                    {"DEB_MAINT_PARAMS": "abort-upgrade"}):
            before = self.snapshot(self.boot)
            result = self.run_update(env=env)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(self.snapshot(self.boot), before)


if __name__ == "__main__":
    unittest.main(verbosity=2)
