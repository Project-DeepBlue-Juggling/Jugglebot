FROM ubuntu:focal

ARG USERNAME="devops"
ARG JUGGLEBOT_REPO_SSH_URL="git@github.com:joewalp/Jugglebot.git"

    # TASK [Upgrade the packages and restore the man command]
RUN export DEBIAN_FRONTEND=noninteractive \
    && apt-get update \
    && apt-get upgrade -y \
    && yes | unminimize 2>&1

ENV LANG="C.UTF-8"
    
    # TASK [Enable the universe respository]
RUN export DEBIAN_FRONTEND=noninteractive \
    && apt-get install -yq --no-install-recommends software-properties-common \
    && add-apt-repository universe \
    && apt-get update

    # TASK [Ensure that build tools, utilities and tini are installed]
RUN export DEBIAN_FRONTEND=noninteractive \
    && apt-get install -yq --no-install-recommends \
      apt-transport-https \
      apt-utils \
      gawk \
      bash-completion \
      build-essential \
      bzip2 \
      ca-certificates \
      clang \
      cmake \
      coreutils \
      cppcheck \
      curl \
      dialog \
      dirmngr \
      expect \
      gdb \
      git \
      gnupg2 \
      grep \
      htop \
      info \
      init-system-helpers \
      iproute2 \
      jq \
      less \
      libc6 \
      libgcc1 \
      libkrb5-3 \
      libgssapi-krb5-2 \
      libicu[0-9][0-9] \
      liblttng-ust[0-9] \
      libsecret-1-dev \
      libstdc++6 \
      lldb \
      llvm \
      locales \
      lsb-release \
      lsof \
      make \
      man-db \
      manpages \
      manpages-dev \
      nano \
      ncdu \
      net-tools \
      netcat \
      openssh-client \
      procps \
      psmisc \
      python-pip-whl \
      python3-dev \
      python3-pip \
      rsync \
      sed \
      strace \
      sudo \
      swig3.0 \
      tini \
      tree \
      unzip \
      vim \
      vim-doc \
      vim-tiny \
      valgrind \
      wget \
      xtail \
      xz-utils \
      zip \
      zlib1g \
    # Clean up
    && apt-get autoremove -y && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Note: We do not install manpages-posix and manpages-posix-dev, which are
# non-free packages in Debian.

    # TASK [Ensure that at least the en_US.UTF-8 UTF-8 locale is available]
RUN if ! grep -o -E '^\s*en_US.UTF-8\s+UTF-8' /etc/locale.gen > /dev/null; then \
      echo 'en_US.UTF-8 UTF-8' >> /etc/locale.gen; \
    fi

VOLUME [ "/home/$USERNAME", "/tmp/.ssh", "/home/$USERNAME/.oh-my-zsh-custom" ]

    # TASK [Create the default user]
RUN groupadd "$USERNAME" \
    && useradd --create-home --shell '/bin/bash' --gid "$USERNAME" --groups 'sudo' "$USERNAME" \
    && echo -n "$USERNAME:$USERNAME" | chpasswd

ENV SHELL='/bin/bash'

WORKDIR "/home/$USERNAME"

USER "$USERNAME"

ENV USER="$USERNAME" 

ADD --keep-git-dir [ "$JUGGLEBOT_REPO_SSH_URL", "/home/$USERNAME" ]

RUN "/home/$USERNAME/Jugglebot/environments/ubuntu-common/base_setup.sh" \
      --jugglebot-conda-env-filepath "/home/$USERNAME/Jugglebot/environments/ubuntu-common/jugglebot_conda_env.yml" \
      --ansible-playbook-command "ANSIBLE_LOCALHOST_WARNING=False ANSIBLE_INVENTORY_UNPARSED_WARNING=False ANSIBLE_BECOME_PASS=$USERNAME ansible-playbook /home/$USERNAME/Jugglebot/environments/ubuntu_20.04-docker-native/main_playbook.yml -e upgrade_software=yes"

ENV SHELL=/usr/bin/zsh GIT_DISCOVERY_ACROSS_FILESYSTEM=1

ENTRYPOINT [ "tini", "--" ]

# CMD [ "sleep", "infinity" ]
CMD [ "/usr/bin/zsh" ]

