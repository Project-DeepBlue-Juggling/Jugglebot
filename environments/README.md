
## Introduction

This is a preview of the environment provisioning project. Currently, the
instructions and scripts encompass provisioning an Ubuntu-24.04 for WSL2 host
that runs the Docker Engine for Linux that has Qemu integration installed and
that has the Qemu arm64 emulator registered. Ansible tasks perform the bulk of
the provisioning. Those tasks could be run on any vanilla Ubuntu host.
Eventually, we will use those tasks to provision a simpler development
environment within a Docker image that can cater to casual contributors.
However, the current phase of the project is focused on building tooling that is
usable within H. Low's workflow on the hardware prototyping environment (aka the
production environment aka Prod).

## Instructions

### Step 1. Ensure that WSL2 is installed

```PowerShell
wsl --install --no-distribution
```

### Step 2. Verify that an Ubuntu-24.04 distribution is not registered.

```PowerShell
wsl --list
```

If the output includes `Ubuntu` or `Ubuntu (Default)`, then run the following
command to determine its version:

```PowerShell
wsl -d Ubuntu -e lsb_release --description
```

If neither of the commands above indicates that some version Ubuntu 24.04 (such
as `Ubuntu-24.04` or `Ubuntu 24.04.1 LTS`) is registered, then you can proceed
to step 2. 

If either of the commands above indicates that you have a prexisting Ubuntu
24.04 distribution, then you have to somehow provision a vanilla Ubuntu 24.04
instance. As of this writing, it seems that Microsoft and Canonical have not
made it easy to acquire a tarball of a vanilla rootfs for Ubuntu 24.04 for WSL.
Below are three options.

#### Fixup Option 1 [RECOMMENDED]. Move your Ubuntu 24.04 instance

This option only relies on the wsl tool. The process looks like this:

1. Shutdown the preexisting distribution
2. Export the preexising distribution to a vhdx or a tarball
3. Import the preexsting distribution from the vxdx or the tarball and give it a
   different distribution name
4. After verifying that the newly imported distribution is healthy, unregister
   the preexisting distribution

Below are some example commands. These assume that the preexisting distribution
is using WSL version 2 and that its virtual disk uses the default ext4 filesystem.

```PowerShell
wsl --terminate <Distribution Name>
wsl --export <Distribution Name> <Vhdx FileName> --vhd
wsl --import-in-place <Alternative Distribution Name> <Vhdx FileName>
wsl --unregister <Distribution Name>
```

#### Fixup Option 2. Use the `download-rootfs` GitHub Action

This option is probably only approachable if you have prior experience with
GitHub CI. The `download-rootfs` GitHub Action
(`Ubuntu/WSL/.github/actions/download-rootfs@main`) can download a vanilla WSL
tarball to a GitHub CI instance. Then, you can scp the tarball to localhost.

#### Fixup Option 3. Build the `release-info` tool from the Ubuntu WSL repo

This option is probably only approachable if you have prior experience with
building a Golang application using PowerShell. The `release-info` tool
(`https://github.com/ubuntu/WSL/tree/main/wsl-builder/release-info`) can
download a manifest that includes a download URL for a vanilla WSL tarball. Note
that the aformentioned `download-rootfs` GitHub Action uses this `release-info`
tool under the hood. The source code for that Action is in the same repo.

### Step 3. Install Ubuntu-24.04

Assuming that you didn't have a preexisting Ubuntu 24.04 instance or that you
used Fixup Option 1, you can now create a fresh Ubuntu 24.04 instance. Run the
following command:

```PowerShell
wsl --install Ubuntu-24.04
```

Eventually, it will prompt you to supply a username and a password. This
username and password don't need to match your Windows username and password.
Typically, a linux username will be all lowercase alphanumeric characters
beginning with an alpha character. If you want a username suggestion, use your
first name in lowercase.

These credentials will not be used to sign-in. Rather, they will be used to
grant access to run privileged system administration commands using sudo.

### Step 4. Provision an ssh keypair

To enable convenient access to GitHub and other remote hosts, we want to use
key-based authentication as described here:

https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent?platform=linux

#### Option 1. Create a new keypair

Substitute your email address into the following command, omitting the brackets.

```Bash
ssh-keygen -t ed25519 -C '[your email address]'
```

That will prompt you to specify a passphrase to secure the key that is being
created. Note that the setup script that you'll run in Step 7 will configure
this host such that you only have to unlock your key once per bootup of the
distribution, so it's okay to choose a lengthy passphrase.

That command will create a private key file and a public key file with the
appropriate permissions in your `~/.ssh` directory.

#### Option 2. Copy a preexisting keypair

You may already have a keypair registered with GitHub that you've been using for
local development. Here are some commands that you could use to copy it into
place if those files are stored in the mounted filesystem:

```Bash
install -m 700 -d ~/.ssh
install -m 600 /mnt/c/[path to private key] -t ~/.ssh
install -m 644 /mnt/c/[path to public key] -t ~/.ssh
```

### Step 5. Add your public key to GitHub

You have two options for how to add your public key to your GitHub account.

#### Option 1 [RECOMMENDED]. Use the GitHub website

See the following instructions:

https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account?platform=windows&tool=webui

#### Option 2 [UNTESTED]. Install the GitHub command line tool and then use it

Add the GitHub CLI package repository and then install the `gh` package as
described here:

https://github.com/cli/cli/blob/trunk/docs/install_linux.md

```Bash
(type -p wget >/dev/null || (sudo apt update && sudo apt-get install wget -y)) \
	&& sudo mkdir -p -m 755 /etc/apt/keyrings \
	&& wget -qO- https://cli.github.com/packages/githubcli-archive-keyring.gpg | sudo tee /etc/apt/keyrings/githubcli-archive-keyring.gpg > /dev/null \
	&& sudo chmod go+r /etc/apt/keyrings/githubcli-archive-keyring.gpg \
	&& echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | sudo tee /etc/apt/sources.list.d/github-cli.list > /dev/null \
	&& sudo apt update \
	&& sudo apt install gh -y
```

Then use `gh` to add your public key to your GitHub account as described here:

https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account?platform=linux&tool=cli

```Bash
gh ssh-key add ~/.ssh/id_ed25519.pub --type authentication --title 'Jugglebot dev env'
```

### Step 6. Clone the Jugglebot repo

```Bash
sudo apt install git

cd ~ && GIT_SSH_COMMAND="ssh -i ${HOME}/.ssh/id_ed25519 -o IdentitiesOnly=yes" git clone git@github.com:Project-DeepBlue-Juggling/Jugglebot.git
```

### Step 7. Run the setup script

informational: This does not access the mounted Windows filesystem.

```Bash
cd ~ && ./Jugglebot/environments/ubuntu_24.04-wsl2/dev-env-setup.sh --ssh-keypair-name id_ed25519
```

### Step 8. Exit and then start a new terminal session to enable all changes

```Bash
exit
```

To start a new terminal session, you have a couple options

#### Option 1 [RECOMMENDED]. Use the Windows Terminal profile

In the dropdown menu of the Windows Terminal tab bar, you'll find an entry for
the newly created distribution.

#### Option 2. Use the wsl tool

```PowerShell
wsl -d Ubuntu-24.04
```

## Notes

This command line environment uses Z Shell (zsh) with Oh My Zsh with the 'clean'
built-in theme. The Python environment is managed by conda and pip rather than
virtualenv and pip because the conda-forge dependency management makes life
easier.

## Noteworthy future milestones

- Demonstrate ROS2 Foxy startup on Ubuntu for WSL2
- Demonstrate ROS2 Foxy startup on arm64 Jetson Linux Docker image emulated by
  Qemu
- Provide instructions for how to configure Visual Studio Code for Windows to
  drive a Linux shell environment and Linux build tools on another host
- Demonstrate how to run a unit test using Visual Studio Code
- Test the usability of Wayland/X11 rendering by WSLg of an app running on
  Ubuntu for WSL2
- Test the usability of Wayland/X11 rendering by WSLg of an app running on arm64
  Jetson Docker image emulated by Qemu
- On a Windows 11 machine with an Nvidia GPU, demonstrate Cuda passthru from an
  arm64 Jetson Linux Docker image
- Provide instructions for how to configure Docker Desktop for Windows to manage
  Docker Engine for Linux for WSL2


