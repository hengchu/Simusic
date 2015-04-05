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
stepDelta = framePeriod / fromIntegral frameSteps

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
                     let w2 = fromIntegral w / 2 :: Double
                     let groundShapeType = H.LineSegment (-w2|+|0) (w2|+|0) 1
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

                     H.setDefaultCollisionHandler space
                       H.Handler {H.beginHandler = Nothing
                                 ,H.preSolveHandler = Nothing
                                 ,H.postSolveHandler = Just handler
                                 ,H.separateHandler = Nothing
                                 }

                     return $ SimST (w, h) space objs collisions

-- Destroys a simST and frees its resources
destroySimST :: SimST -> IO ()
destroySimST st = H.freeSpace $ stSpace st

class PhysicalObjDef a where
  mass :: a -> H.Mass
  moment :: a -> H.Moment
  pos :: a -> H.Position
  ela :: a -> H.Elasticity
  shape :: a -> H.ShapeType

-- Specification of a ball object
data BallDef = BallDef {ballPos::H.Position
                       ,ballRadius::H.Distance
                       ,ballElasticity::H.Elasticity
                       ,ballMass::H.Mass
                       ,ballMoment::H.Moment
                       }

instance PhysicalObjDef BallDef where
  mass = ballMass
  moment = ballMoment
  pos = ballPos
  ela = ballElasticity
  shape (BallDef {ballRadius=radius}) = H.Circle radius

data SquareDef = SquareDef {squarePos::H.Position
                           ,squareSideLength::H.Distance
                           ,squareElasticity::H.Elasticity
                           ,squareMass::H.Mass
                           ,squareMoment::H.Moment}

instance PhysicalObjDef SquareDef where
  mass = squareMass
  moment = squareMoment
  pos = squarePos
  ela = squareElasticity
  shape (SquareDef {squareSideLength=sidelength}) = 
    let sidelen2 = sidelength / 2
        vertices = [ H.Vector (-sidelen2) (-sidelen2)
                   , H.Vector (-sidelen2) sidelen2
                   , H.Vector sidelen2 sidelen2
                   , H.Vector sidelen2 (-sidelen2)]
    in  H.Polygon vertices

addObj :: (PhysicalObjDef a) => a -> S.StateVar SimST -> IO ()
addObj def st = do
  st' <- S.get st
  body <- H.newBody (mass def) (moment def)
  let shapetype = shape def
  shp <- H.newShape body shapetype 0
  H.elasticity shp S.$= ela def
  H.position body S.$= pos def

  H.spaceAdd (stSpace st') body
  H.spaceAdd (stSpace st') shp

  let objs = stObjects st'
  let bodyobj = O body [(shp, shapetype)]
  let remote = do H.spaceRemove (stSpace st') body
                  H.spaceRemove (stSpace st') shp
  let objs' = M.insert bodyobj remote objs

  st S.$= st'{stObjects=objs'}

-- Decorated function for UISF
uisfAddObj :: (PhysicalObjDef a) => a -> S.StateVar SimST -> IO ()
uisfAddObj = addObj

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
     newst <- foldM (foldhelper $ fromIntegral w) objmap $ M.assocs objmap
     st S.$= st'{stObjects=newst}
  where
    foldhelper width objs (o@(O body _), remove) = do
      H.Vector x y <- S.get $ H.position body
      if y < (-350) || abs x > (width+400)
         then remove >> return (M.delete o objs)
         else return objs
                         
instance Simulatable SimST where
  advance st = do removeOutOfSight st
                  st' <- S.get st
                  let step = H.step (stSpace st') stepDelta
                  replicateM_ frameSteps step


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
       v@(H.Vector x0 y0)
       (_, (w, h)) =
      let (cx, cy) = convCoord (w, h) (round x0, round y0)
          bodydot = withColor Black $ ellipse (cx-2, cy-2) (cx+2, cy+2)
          shapes = map (draw' v . snd) shapesAndShapeTypes
      in  foldr overGraphic nullGraphic $ bodydot:shapes
    where draw' :: H.Position -> H.ShapeType -> Graphic
          draw' (H.Vector x y) shapetype =
            let (xo, yo) = convCoord (w, h) (round x, round y)
                graphic = case shapetype of
                  H.Circle r -> let upperleft = (xo - round r, yo - round r)
                                    lowerright = (xo + round r, yo + round r)
                                in  withColor' defColor $ ellipse upperleft lowerright
                  H.LineSegment start end _ ->
                                let H.Vector sx sy = start
                                    H.Vector ex ey = end
                                    startUISF = convCoord (w, h) (round (x+sx), round (y+sy))
                                    endUISF = convCoord (w, h) (round (x+ex), round (y+ey))
                                in FRP.UISF.SOE.line startUISF endUISF
                  H.Polygon vertices ->
                                let vertices' = map (+ H.Vector x y) vertices
                                    verticesUISF = map (\(H.Vector x1 y1) -> convCoord (w, h) (round x1, round y1)) vertices'
                                in  withColor' defColor $ polygon verticesUISF
            in  graphic
          defColor = rgba (100::Int) (100::Int) (100::Int) (80::Int)

-- Position for each object in Hipmunk coordinates
type WState = [(Object, H.Position)]

presentALayout :: Layout
presentALayout = makeLayout (Fixed $ fst worldSize) (Fixed $ snd worldSize)

presentACompute :: WState -> WState -> Rect -> UIEvent -> ((), WState, DirtyBit)
presentACompute input0 st _ _ = ((), input0, input0 /= st)

presentADraw :: Rect -> Bool -> WState -> Graphic
presentADraw rect _ state =
  let graphics = map (flip (uncurry draw) rect) state
  in  foldr overGraphic nullGraphic graphics

-- This is the widget used to present the simulation.
presentA :: UISF WState ()
presentA = mkWidget [] presentALayout presentACompute presentADraw

uisfUpdateST :: S.StateVar SimST -> IO WState
uisfUpdateST st = do
  advance st
  st' <- S.get st
  let objs = M.keys $ stObjects st'
  let maphelper (O body _) = S.get $ H.position body
  positions <- mapM maphelper objs
  return $ zip objs positions

simulationA :: UISF (S.StateVar SimST) WState
simulationA = uisfPipe uisfUpdateST

addObjA :: (PhysicalObjDef a) => UISF (SEvent (a, S.StateVar SimST)) (SEvent ())
addObjA = uisfSinkE (uncurry uisfAddObj)

parserSelect :: [(a, String)] -> a -> a
parserSelect (r:_) _ = fst r
parserSelect _ def = def

addCommonA :: UISF () (H.Mass, H.Moment, Double, Double, H.Elasticity)
addCommonA = leftRight $ proc _ -> do
  masstext   <- label "Mass" >>> textboxE "1.0"       -< Nothing
  momenttext <- label "M.Inertia" >>> textboxE "5.0"  -< Nothing
  xtext      <- label "x" >>> textboxE "400"          -< Nothing
  ytext      <- label "y" >>> textboxE "800"          -< Nothing
  elatext    <- label "Elasticity" >>> textboxE "0.9" -< Nothing

  let massv   = flip parserSelect 1.0 $ reads masstext :: Double
  let momentv = flip parserSelect 5.0 $ reads momenttext :: Double
  let x      = flip parserSelect 400 $ reads xtext :: Double
  let y      = flip parserSelect 800 $ reads ytext :: Double
  let elav    = flip parserSelect 1.0 $ reads elatext :: Double

  returnA -< (massv, momentv, x, y, elav)

addBallBtnA :: UISF () (SEvent BallDef)
addBallBtnA = leftRight $ proc _ -> do
  (massv, momentv, x, y, elav) <- addCommonA                       -< ()
  radiustext                <- label "Radius" >>> textboxE "10" -< Nothing
  let radius = flip parserSelect 10 $ reads radiustext :: Double

  let balldef = BallDef (x|+|y) radius elav massv momentv

  click      <- button "AddBall" -< ()
  clicked    <- edge             -< click
  returnA                        -< fmap (const balldef) clicked

addSquareBtnA :: UISF () (SEvent SquareDef)
addSquareBtnA = leftRight $ proc _ -> do
  (massv, momentv, x, y, elav) <- addCommonA                     -< ()
  sidetext                  <- label "Side" >>> textboxE "10" -< Nothing
  let side   = flip parserSelect 10 $ reads sidetext :: Double

  let squaredef = SquareDef (x|+|y) side elav massv momentv

  click   <- button "AddSquare" -< ()
  clicked <- edge               -< click
  returnA                       -< fmap (const squaredef) clicked

collisionsA :: UISF (S.StateVar SimST) [Collision]
collisionsA = uisfPipe extractCollision


collision2midi :: Collision -> MidiMessage
collision2midi (H.Vector x y, vel) = ANote (round (interpolate 1200 x 16) `mod` 16)
                                           (round (interpolate 800 y 127) `mod` 127)
                                           (round (interpolate 500 vel 127) `mod` 127)
                                           1.0
  -- Interpolate a val bwteen [0, max] to a value in [0, range]
  where interpolate max0 val range = val * (range / max0)

present :: S.StateVar SimST -> UISF () ()
present st = proc _ -> do
  devid              <- selectOutput                     -< ()
  squaredef          <- addSquareBtnA                    -< ()
  balldef            <- addBallBtnA                      -< ()
  _                  <- addObjA <<< delay Nothing        -< maybe Nothing (\bd -> Just (bd, st)) balldef
  _                  <- addObjA <<< delay Nothing        -< maybe Nothing (\bd -> Just (bd, st)) squaredef
  wstate             <- simulationA                      -< st
  collisions         <- collisionsA                      -< st
  collisionsuniq     <- unique                           -< collisions
  _                  <- midiOut                          -< (devid, fmap (map collision2midi) collisionsuniq)
  presentA                                               -< wstate
  
main :: IO ()
main = do st <- newSimST worldSize
          stref <- newIORef st
          let sst          = S.makeStateVar (readIORef stref) (writeIORef stref)
          let defaultClose = uiClose defaultUIParams
          let myClose      = destroySimST st >> defaultClose
          runMUI defaultUIParams{uiSize=worldSize, uiClose=myClose} (present sst)
