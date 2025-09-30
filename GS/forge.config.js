module.exports = {
  packagerConfig: {
    asar: true, // Bundles the app in a single ASAR file for distribution
    icon: "./build/Parachute", // Path to the app icon without extension; Electron will auto-detect the platform-specific format (.ico for Windows, .icns for macOS, .png for Linux)
  },
  rebuildConfig: {},
  makers: [
    // {
    //   name: "@electron-forge/maker-squirrel",
    //   config: {
    //     authors: "EdgeFlyte",
    //     description: "EF CubeSat GTerminal V1",
    //     iconUrl: "https://edgeflyte.com/img/favicon.ico",
    //     setupIcon: "./build/Parachute.ico", // Windows-specific icon for the installer
    //     exe: "EFCSGTerm.exe", // Executable name for Windows
    //     name: "EFCSGTerm", // App's name for Windows
    //   },
    // },
    // {
    //   name: "@electron-forge/maker-zip",
    //   platforms: ["darwin", "linux", "win32"], // Supports zip packaging for all platforms
    // },
    {
      name: "@electron-forge/maker-deb",
      config: {
        options: {
          maintainer: "EdgeFlyte",
          homepage: "https://edgeflyte.com",
        },
      },
    },
    {
      name: "@electron-forge/maker-rpm",
      config: {
        options: {
          license: "MIT",
          description: "EF CubeSat GTerminal V1",
        },
      },
    },
    {
      name: "@electron-forge/maker-generic",
      config: {
        platforms: ["linux"],
        targets: ["AppImage"], // Specify AppImage as the target
        options: {
          name: "EF CubeSat GTerminal",
          icon: "./build/Parachute.png", // AppImage-specific icon
        },
      },
    },
  ],
  plugins: [
    {
      name: "@electron-forge/plugin-auto-unpack-natives",
      config: {},
    },
  ],
};
