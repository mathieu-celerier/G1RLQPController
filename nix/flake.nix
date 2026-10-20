{
  description = "Build shell for G1RLQPController with the mc_rtc stack pinned to Raphaël Barrabes's 2026-08-13 workspace";

  # The mc_rtc stack (mc_rtc, tvm, Tasks, state-observation) and the G1 robot module are built by Nix
  # from the commits in ../deploy/pinned-commits.txt. mc_mujoco (Bastien Muraccioli's fork with
  # external torques), g1_mj_description, mc_external_forces_observer, mc_joystick_plugin and the
  # controller are built from source inside the shell (see ../nix/README.md).
  inputs = {
    mc-rtc-nix.url = "github:mc-rtc/nixpkgs/cf141659f69f8cd37256d13e202ccbec90458ca7";
    flake-parts.follows = "mc-rtc-nix/flake-parts";
    systems.follows = "mc-rtc-nix/systems";

    mc-rtc-src = {
      url = "git+https://github.com/bastien-muraccioli/mc_rtc?rev=9aae9c4745eec409572eb631aa3fbce17e5cdde2&submodules=1";
      flake = false;
    };
    tvm-src = {
      url = "git+https://github.com/bastien-muraccioli/tvm?rev=62aacd80d49a11e2d56dd3a7dcf712340f7f6b68&submodules=1";
      flake = false;
    };
    tasks-src = {
      url = "git+https://github.com/bastien-muraccioli/Tasks?rev=827e136322da3e2f1d9c28141b00fe43a9f35a4a&submodules=1";
      flake = false;
    };
    state-observation-src = {
      url = "git+https://github.com/bastien-muraccioli/state-observation?rev=1b319a1e727022f28d2f82c1535c3d9f9207e608&submodules=1";
      flake = false;
    };
    # isri-aist/mc_g1 7e35e3f + Raphaël's rotor inertia / gear ratio change
    mc-g1-src = {
      url = "git+https://github.com/mathieu-celerier/mc_g1?ref=topic/raphael-live-2026-08-13&rev=32377806658eac020e6f0751a38e001fa1ca71c9&submodules=1";
      flake = false;
    };
    # isri-aist/g1_description e4fa30f + Raphaël's URDF effort limits
    g1-description-src = {
      url = "git+https://github.com/mathieu-celerier/g1_description?ref=topic/raphael-live-2026-08-13&rev=685f0a63b2ba65934406d993a307bdc40b3a7839&submodules=1";
      flake = false;
    };
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } (
      { lib, ... }:
      {
        systems = [ "x86_64-linux" ];
        imports = [
          inputs.mc-rtc-nix.flakeModule
          {
            mc-rtc-nix = {
              with-ros = false;
              with-python = false;
            };
            flakoboros.overrideAttrs = {
              mc-rtc = { drv-prev, ... }: {
                src = inputs.mc-rtc-src;
                version = "2.15.2-bastien";
                doCheck = false;
              };
              tvm = { ... }: {
                src = inputs.tvm-src;
                doCheck = false;
              };
              tasks-qld = { ... }: {
                src = inputs.tasks-src;
                doCheck = false;
              };
              state-observation = { ... }: {
                src = inputs.state-observation-src;
                doCheck = false;
              };
              mc-g1 = { ... }: { src = inputs.mc-g1-src; };
              g1-description = { drv-prev, ... }: {
                src = inputs.g1-description-src;
                # mc_rtc is built without ROS here, so package:// URIs cannot be resolved:
                # point them at the installed share directory instead.
                postInstall = (drv-prev.postInstall or "") + ''
                  find $out -name '*.urdf' -exec sed -i \
                    "s|package://g1_description/|$out/share/mc_rtc/g1_description/|g" {} +
                '';
              };
            };
          }
        ];
        perSystem =
          { pkgs, ... }:
          {
            devShells.default = pkgs.mkShell {
              name = "g1-rl-qp";
              inputsFrom = [ pkgs.mc-rtc ];
              packages = with pkgs; [
                mc-rtc
                mc-g1
                cmake
                ninja
                pkg-config
                cli11
                glew
                glfw3
                libGL
                xorg.libX11
                xorg.libXrandr
                xorg.libXinerama
                xorg.libXcursor
                xorg.libXi
                xorg.libXext
                onnxruntime
              ];
              shellHook = ''
                export G1_WS=''${G1_WS:-$HOME/devel/mc_rtc_nix_ws}
                export CMAKE_PREFIX_PATH=$G1_WS/install:$CMAKE_PREFIX_PATH
                export LD_LIBRARY_PATH=$G1_WS/install/lib64:$G1_WS/install/lib:$LD_LIBRARY_PATH
                export PATH=$G1_WS/install/bin:$PATH
                echo "mc_rtc: ${pkgs.mc-rtc}"
                echo "mc_g1:  ${pkgs.mc-g1}"
                echo "install prefix: $G1_WS/install"
              '';
            };
          };
      }
    );
}
