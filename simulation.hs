{-# LANGUAGE Arrows #-}

import qualified Physics.Hipmunk as H
import qualified Data.Map as M
import qualified Data.StateVar as S
import FRP.UISF.UITypes
import FRP.UISF.SOE
import FRP.UISF.Widget
import FRP.UISF.UISF
import FRP.UISF
import Data.IORef
import Control.Monad
import Control.Monad.IO.Class
import Euterpea.IO.MUI
import Euterpea

desiredFPS :: Int
desiredFPS = 60

framePeriod :: Time
framePeriod = 1 / fromIntegral desiredFPS

-- How many steps should each frame perform
frameSteps :: Int
frameSteps = 6

stepDelta :: H.Time
stepDelta = framePeriod / (fromIntegral frameSteps)

worldSize :: Dimension
worldSize = (1200, 800)

(|+|) :: H.CpFloat -> H.CpFloat -> H.Vector
(|+|) = H.Vector
infix 4 |+|

-- An object in the simulation world
data Object  = O H.Body [(H.Shape, H.ShapeType)]
  deriving (Eq)

instance Ord Object where
  compare (O b1 _) (O b2 _) = compare b1 b2

-- Information about collisions
type Collision = (H.Position
                 ,Double -- Relative velocity of collision
                 )

-- Main state maintained for simulations
data SimST = SimST {stDimension :: Dimension           -- World size
                   ,stSpace :: H.Space                 -- Space object from Hipmunk
                   ,stObjects :: M.Map Object (IO ())  -- IO Action to remove the object
                   ,stCollisions :: IORef [Collision]  -- Collisions detected during last advance
                   }

-- Builds a new default simST with specified world size
newSimST :: Dimension -> IO SimST
newSimST (w, h) = do space <- H.newSpace
                     H.gravity space S.$= 0 |+| -230

                     ground <- H.newBody H.infinity H.infinity
                     let w2 = (fromIntegral w)/2 :: Double
                     let groundShapeType = (H.LineSegment (-w2|+|0) (w2|+|0) 1)
                     groundshape <- H.newShape ground groundShapeType  $ 0|+|0
                     H.elasticity groundshape S.$= 1.0
                     H.position ground S.$= w2 |+| 0
                     H.spaceAdd space (H.Static groundshape)

                     let groundobj = O ground [(groundshape, groundShapeType)]
                     let objs = M.insert groundobj (H.spaceRemove space groundshape) M.empty

                     collisions <- newIORef []
                     let handler = do ps <- H.points
                                      (s1, s2) <- H.shapes
                                      let b1 = H.body s1
                                      let b2 = H.body s2
                                      v1 <- liftIO $ S.get $ H.velocity b1
                                      v2 <- liftIO $ S.get $ H.velocity b2
                                      liftIO $ modifyIORef collisions ((head ps, abs $ H.len $ v2-v1):)

                     H.setDefaultCollisionHandler space $
                       H.Handler {H.beginHandler = Nothing
                                 ,H.preSolveHandler = Nothing
                                 ,H.postSolveHandler = Just handler
                                 ,H.separateHandler = Nothing
                                 }

                     return $ SimST (w, h) space objs collisions

-- Destroys a simST and frees its resources
destroySimST :: SimST -> IO ()
destroySimST st = H.freeSpace $ stSpace st

-- Specification of a ball object
data BallDef = BallDef {ballPos::H.Position
                       ,ballRadius::H.Distance
                       ,ballElasticity::H.Elasticity
                       ,ballMass::H.Mass
                       ,ballMoment::H.Moment
                       }

-- Adds a ball given a BallDef to the simST
addBall :: BallDef -> S.StateVar SimST -> IO (S.StateVar SimST)
addBall (BallDef pos radius elasticity mass minertia) st = do
  st' <- S.get st
  ball <- H.newBody mass minertia
  let ballshapetype = (H.Circle radius)
  ballshape <- H.newShape ball ballshapetype 0
  H.elasticity ballshape S.$= elasticity
  H.position ball S.$= pos
  H.spaceAdd (stSpace st') ball
  H.spaceAdd (stSpace st') ballshape
  
  let objs = stObjects st'
  let ballobj = O ball [(ballshape, ballshapetype)]
  let remove = do H.spaceRemove (stSpace st') ball
                  H.spaceRemove (stSpace st') ballshape
  let objs' = M.insert ballobj remove objs

  st S.$= st'{stObjects=objs'}
  return st

-- Decorated function for UISF
uisfAddBall :: SEvent BallDef -> S.StateVar SimST -> IO (S.StateVar SimST)
uisfAddBall (Just bd) st = addBall bd st
uisfAddBall _ st = return st

extractCollision :: S.StateVar SimST -> IO [Collision]
extractCollision st = do
  st' <- S.get st
  collisions <- readIORef $ stCollisions st'
  writeIORef (stCollisions st') []
  return collisions

class Simulatable a where
  advance :: S.StateVar a -> IO ()

-- Removes objects that are out of the visible boundary
removeOutOfSight :: S.StateVar SimST -> IO ()
removeOutOfSight st = 
  do st' <- S.get st
     let (w, _) = stDimension st'
     let objmap = stObjects st'
     newst <- foldM (f $ fromIntegral w) objmap $ M.assocs objmap
     st S.$= st'{stObjects=newst}
  where
    f width objs (o@(O b _), remove) = do
      H.Vector x y <- S.get $ H.position b
      if y < (-350) || abs x > (width+400)
         then remove >> return (M.delete o objs)
         else return objs
                         
instance Simulatable SimST where
  advance st = do removeOutOfSight st
                  st' <- S.get st
                  let step = H.step (stSpace st') stepDelta
                  replicateM_ (frameSteps) (step)


{-
 - UISF Coordinate:
 - (0, 0) ---------> x grows
 - |     |         |
 - |     |         |
 - |-----.         |
 - |               |
 - |               |
 - |               |
 - |               |
 - v---------------.
 - y grows
 -
 - Hipmunk Coordinate:
 - y grows
 - ^---------------.
 - |               |
 - |               |
 - |               |
 - |               |
 - |               |
 - |               |
 - (0, 0) ---------> x grows
 -}

-- Convert a point between UISF and Hipmunk coordinate systems
convCoord :: Dimension -> Point -> Point
convCoord (_, h) (x, y) = (x, h-y)

class Drawable a where
  draw :: a -> H.Position -> Rect -> Graphic

instance Drawable Object where
  draw (O _ shapesAndShapeTypes)
       v@(H.Vector x y)
       (_, (w, h)) =
      let (cx, cy) = convCoord (w, h) (round x, round y)
          bodydot = withColor Black $ ellipse (cx-2, cy-2) (cx+2, cy+2)
          shapes = map (draw' v) $ map snd shapesAndShapeTypes
      in  foldr overGraphic nullGraphic $ bodydot:shapes
    where draw' :: H.Position -> H.ShapeType -> Graphic
          draw' (H.Vector x y) shapetype =
            let (xo, yo) = convCoord (w, h) (round x, round y)
                graphic = case shapetype of
                  H.Circle r -> let upperleft = (xo - round r, yo - round r)
                                    lowerright = (xo + round r, yo + round r)
                                in  withColor' (rgba 100 100 100 80) $ ellipse upperleft lowerright
                  H.LineSegment start end thickness ->
                                let H.Vector sx sy = start
                                    H.Vector ex ey = end
                                    s = convCoord (w, h) (round (x+sx), round (y+sy))
                                    e = convCoord (w, h) (round (x+ex), round (y+ey))
                                in FRP.UISF.SOE.line s e
                  H.Polygon vertices ->
                                undefined
            in  graphic

-- Position for each object in Hipmunk coordinates
type WState = [(Object, H.Position)]

presentALayout :: Layout
presentALayout = makeLayout (Fixed $ fst worldSize) (Fixed $ snd worldSize)

presentACompute :: WState -> WState -> Rect -> UIEvent -> ((), WState, DirtyBit)
presentACompute input st _ _ = ((), input, input /= st)

presentADraw :: Rect -> Bool -> WState -> Graphic
presentADraw rect _ state =
  let graphics = map (flip (uncurry draw) rect) state
  in  foldr overGraphic nullGraphic graphics

-- This is the widget used to present the simulation.
presentA :: UISF WState ()
presentA = mkWidget [] presentALayout presentACompute presentADraw

uisfUpdateST :: S.StateVar SimST -> IO (S.StateVar SimST, WState)
uisfUpdateST st = do
  advance st
  st' <- S.get st
  let objs = M.keys $ stObjects st'
  let f (O b _) = S.get $ H.position b
  positions <- mapM f objs
  return (st, zip objs positions)

simulationA :: UISF (S.StateVar SimST) (S.StateVar SimST, WState)
simulationA = uisfPipe uisfUpdateST

addBallA :: UISF (SEvent BallDef, S.StateVar SimST) (S.StateVar SimST)
addBallA = uisfPipe (uncurry uisfAddBall)

parserSelect :: [(a, String)] -> a -> a
parserSelect (r:_) _ = fst r
parserSelect _ def = def

uisfShouldAddBall :: (SEvent Bool, BallDef) -> SEvent BallDef
uisfShouldAddBall (Just True, bd) = Just bd
uisfShouldAddBall _ = Nothing

addBallBtnA :: UISF () (SEvent BallDef)
addBallBtnA = leftRight $ proc _ -> do
  masstext   <- label "Mass" >>> textboxE "1.0"       -< Nothing
  momenttext <- label "M.Inertia" >>> textboxE "5.0"  -< Nothing
  xtext      <- label "x" >>> textboxE "400"          -< Nothing
  ytext      <- label "y" >>> textboxE "800"          -< Nothing
  elatext    <- label "Elasticity" >>> textboxE "0.9" -< Nothing
  radiustext <- label "Radius" >>> textboxE "10"      -< Nothing

  let mass   = flip parserSelect 1.0 $ reads masstext :: Double
  let moment = flip parserSelect 5.0 $ reads momenttext :: Double
  let x      = flip parserSelect 400 $ reads xtext :: Double
  let y      = flip parserSelect 800 $ reads ytext :: Double
  let ela    = flip parserSelect 1.0 $ reads elatext :: Double
  let radius = flip parserSelect 10 $ reads radiustext :: Double

  let balldef = BallDef (x|+|y) radius ela mass moment

  clicked               <- button "AddBall" -< ()
  clickchange           <- unique           -< clicked
  arr uisfShouldAddBall                     -< (clickchange, balldef)

collisionsA :: UISF (S.StateVar SimST) [Collision]
collisionsA = uisfPipe extractCollision


collision2midi :: Collision -> MidiMessage
collision2midi (H.Vector x y, velocity) = ANote ((round $ interpolate 1200 x 16) `mod` 16)
                                                ((round $ interpolate 800 y 127) `mod` 127)
                                                ((round $ interpolate 500 velocity 127) `mod` 127)
                                                (1.0)
  -- Interpolate a val bwteen [0, max] to a value in [0, range]
  where interpolate max val range = val * (range / max)

uisfPrint :: (Show a) => Maybe a -> IO ()
uisfPrint (Just s) = print s
uisfPrint Nothing = return ()

present :: S.StateVar SimST -> UISF () ()
present st = proc _ -> do
  devid              <- selectOutput                     -< ()
  balldef            <- addBallBtnA                      -< ()
  rec st1            <- addBallA <<< delay (Nothing, st) -< (balldef, st')
      (st', wstate)  <- simulationA                      -< st1
  collisions         <- collisionsA                      -< st1
  collisionsuniq     <- unique                           -< collisions
  _                  <- uisfSink uisfPrint               -< collisionsuniq
  _                  <- midiOut                          -< (devid, fmap (map collision2midi) collisionsuniq)
  presentA                                               -< wstate
  
main :: IO ()
main = do st <- newSimST worldSize
          stref <- newIORef st
          let sst = S.makeStateVar (readIORef stref) (writeIORef stref)
          runMUI defaultUIParams{uiSize=worldSize} (present sst)
