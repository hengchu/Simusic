# Simusic
An interactive demo written in Haskell with Euterpea and Hipmunk

# How to run it
You would need to `cabal install Euterpea`, and install the version of Hipmunk here: https://github.com/hengchu/Hipmunk

For some reason the 5.x version of Chipmunk would crash when I use UISF with it.

Then you can run

```
cabal install --dependencies-only
cabal configure
cabal build
./dist/build/simulation/simulation
```
