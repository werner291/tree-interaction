let
  moz_overlay = import (builtins.fetchTarball https://github.com/mozilla/nixpkgs-mozilla/archive/master.tar.gz);
  nixpkgs = import <nixpkgs> { overlays = [ moz_overlay ]; };
  rust = (nixpkgs.rustChannelOf { channel = "stable"; }).rust.override {
    extensions = [ "rust-src" "rust-analysis" "rls-preview" ];
    };
in
  with nixpkgs;
  stdenv.mkDerivation {
    name = "rust";
    
    nativeBuildInputs = [
        pkgconfig
        python3
    ];

    buildInputs = [
	    rust  gtk3 libGL cmake dhall openssl SDL2 SDL2_image llvmPackages.libclang
    ];

    LD_LIBRARY_PATH = with pkgs.xlibs; "${pkgs.libGL}/lib";
    LIBCLANG_PATH = "${pkgs.llvmPackages.libclang}/lib";

  }
