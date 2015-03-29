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

data Object  = O H.Body [(H.Shape, H.ShapeType)]
  deriving (Eq)

instance Ord Object where
  compare (O b1 _) (O b2 _) = compare b1 b2

type Collision = (H.Position
                 ,Double -- Relative velocity of collision
                 )

data SimST = SimST {stDimension :: Dimension
                   ,stSpace :: H.Space
                   ,stObjects :: M.Map Object (IO ())  -- IO Action to remove the object
                   ,stCollisions :: IORef [Collision] -- Collisions detected during last step
                   }

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

                     let lineobj = O ground [(groundshape, groundShapeType)]
                     let objs = M.insert lineobj (return ()) M.empty

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

destroySimST :: SimST -> IO ()
destroySimST st = H.freeSpace $ stSpace st

data BallDef = BallDef {ballPos::H.Position
                       ,ballRadius::H.Distance
                       ,ballElasticity::H.Elasticity
                       ,ballMass::H.Mass
                       ,ballMoment::H.Moment
                       }

addBall :: SEvent BallDef -> S.StateVar SimST -> IO (S.StateVar SimST)
addBall (Just (BallDef pos radius elasticity mass minertia)) st = do
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
addBall Nothing st = return st

extractCollision :: S.StateVar SimST -> IO [Collision]
extractCollision st = do
  st' <- S.get st
  collisions <- readIORef $ stCollisions st'
  writeIORef (stCollisions st') []
  return collisions

class Simulatable a where
  advance :: S.StateVar a -> IO ()

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
                  let clearCollisions = writeIORef (stCollisions st') []
                      step = H.step (stSpace st') stepDelta
                  replicateM_ (frameSteps) (step)
                  --step


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

convCoord :: Dimension -> Point -> Point
convCoord (_, h) (x, y) = (x, h-y)

(|+|) :: H.CpFloat -> H.CpFloat -> H.Vector
(|+|) = H.Vector
infix 4 |+|

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

type WState = [(Object, H.Position)] -- Position for each object in Hipmunk coordinates

worldSize :: Dimension
worldSize = (1200, 800)

wLayout :: Layout
wLayout = makeLayout (Fixed $ fst worldSize) (Fixed $ snd worldSize)

wCompute :: WState -> WState -> Rect -> UIEvent -> ((), WState, DirtyBit)
wCompute input st _ _ = ((), input, input /= st)

wDraw :: Rect -> Bool -> WState -> Graphic
wDraw rect _ state =
  let graphics = map (flip (uncurry draw) rect) state
  in  foldr overGraphic nullGraphic graphics

-- This is the widget used to present the simulation.
wWidget :: UISF WState ()
wWidget = mkWidget [] wLayout wCompute wDraw

updateST :: S.StateVar SimST -> IO (S.StateVar SimST, WState)
updateST st = do
  advance st
  st' <- S.get st
  let objs = M.keys $ stObjects st'
  let f (O b _) = S.get $ H.position b
  positions <- mapM f objs
  return (st, zip objs positions)

sWidget :: UISF (S.StateVar SimST) (S.StateVar SimST, WState)
sWidget = uisfPipe updateST

addBallWidget :: UISF (SEvent BallDef, S.StateVar SimST) (S.StateVar SimST)
addBallWidget = uisfPipe (uncurry addBall)

parserSelect :: [(a, String)] -> a -> a
parserSelect (r:_) _ = fst r
parserSelect _ def = def

addBallButton :: UISF () (SEvent BallDef)
addBallButton = leftRight $ proc _ -> do
  masstext <- label "Mass" >>> textboxE "1.0" -< Nothing
  momenttext <- label "M.Inertia" >>> textboxE "5.0" -< Nothing
  xtext <- label "x" >>> textboxE "400" -< Nothing
  ytext <- label "y" >>> textboxE "800" -< Nothing
  elatext <- label "Elasticity" >>> textboxE "0.9" -< Nothing
  radiustext <- label "Radius" >>> textboxE "10" -< Nothing

  let mass   = flip parserSelect 1.0 $ reads masstext :: Double
  let moment = flip parserSelect 5.0 $ reads momenttext :: Double
  let x      = flip parserSelect 400 $ reads xtext :: Double
  let y      = flip parserSelect 800 $ reads ytext :: Double
  let ela    = flip parserSelect 1.0 $ reads elatext :: Double
  let radius = flip parserSelect 10 $ reads radiustext :: Double

  clicked <- button "AddBall" -< ()
  clickchange <- unique -< clicked
  let balldef = BallDef (x|+|y) radius ela mass moment
  arr shouldAddBall -< (clickchange, balldef)

shouldAddBall :: (SEvent Bool, BallDef) -> SEvent BallDef
shouldAddBall (Just True, bd) = Just bd
shouldAddBall _ = Nothing

extractCollisionWidget :: UISF (S.StateVar SimST) [Collision]
extractCollisionWidget = uisfPipe extractCollision

print' :: (Show a) => Maybe a -> IO ()
print' (Just s) = print s
print' Nothing = return ()

interpolate :: Double -> Double -> Double -> Double
interpolate max val range = val * (range / max)

collision2midi :: Collision -> MidiMessage
collision2midi (H.Vector x y, velocity) = ANote ((round $ interpolate 1200 x 16) `mod` 16)
                                                ((round $ interpolate 800 y 127) `mod` 127)
                                                ((round $ interpolate 500 velocity 127) `mod` 127)
                                                (1.0)

present :: S.StateVar SimST -> UISF () ()
present st = proc _ -> do
  devid <- selectOutput -< ()
  balldef <- addBallButton -< ()
  rec st1 <- addBallWidget <<< delay (Nothing, st) -< (balldef, st')
      (st', wstate) <- sWidget -< st1
  collisions <- extractCollisionWidget -< st1
  collisionsuniq <- unique -< collisions
  _ <- uisfSink print' -< collisionsuniq
  _ <- midiOut -< (devid, fmap (map collision2midi) collisionsuniq)
  wWidget -< wstate
  
main :: IO ()
main = do st <- newSimST worldSize
          stref <- newIORef st
          let sst = S.makeStateVar (readIORef stref) (writeIORef stref)
          runMUI defaultUIParams{uiSize=worldSize} (present sst)
