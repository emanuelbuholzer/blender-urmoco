blender-urmoco is supported on Ubuntu 24.04.1 LTS Noble Numbat and deprecated 
on Ubuntu 22.04.5 LTS Jammy Jellyfish.

### Ubuntu Jammy Jellyfish

Running blender-urmoco with Ubuntu Jammy requires installing Blender manually.
We test and by default run Blender 3.6 LTS.

If you're developing using distrobox, create and enter ubuntu-jammy:
```bash
distrobox assemble create --name ubuntu-jammy
distrobox enter ubuntu-jammy
```

If you're running a machine with Ubuntu, ensure the prerequisites for Blender are installed:
```bash
sudo apt-get update && \
  sudo apt-get install libsm6
```

Go to the [Blender 3.6 LTS Download Page](https://www.blender.org/download/lts/3-6/)
and download the Linux version. 
Once you've downloaded it, unpack it:
```bash
tar -xf blender-3.6.16-linux-x64.tar.xz && \
  mv blender-3.6.16-linux-x64 blender && \
  rm blender-3.6.16-linux-x64.tar.xz
```

Open blender to verify it is installed correctly and can be used:
```bash
blender/blender
```

