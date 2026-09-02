import asyncio, json, time, struct, msgpack, websockets, math, pickle

URI="ws://127.0.0.1:5810/nt/demorec"; SUB=["v4.1.networktables.first.wpi.edu","networktables.first.wpi.edu"]
TID={"double[]":17,"string":4,"double":1}

TOPICS=["/AIControl/","/AdvantageKit/RealOutputs/FieldSimulation/","/AdvantageKit/RealOutputs/Sim/Fuel/",
        "/AdvantageKit/RealOutputs/LaunchCalculator/","/AdvantageKit/RealOutputs/Odometry/"]

def dbls(b):
    if isinstance(b,(bytes,bytearray)) and len(b)>=8:
        n=len(b)//8
        return list(struct.unpack("<%dd"%n, b[:n*8]))
    return []

class Rec:
    def __init__(self): self.frames=[]; self.caption=""; self.prompt=""

async def main():
    rec=Rec()
    async with websockets.connect(URI, subprotocols=SUB, max_size=None) as ws:
        subs=[{"method":"subscribe","params":{"topics":TOPICS,"subuid":1,"options":{"prefix":True,"all":True,"periodic":0.03}}},
              {"method":"publish","params":{"name":"/AIControl/TargetPose","pubuid":10,"type":"double[]","properties":{}}},
              {"method":"publish","params":{"name":"/AIControl/ActionTrigger","pubuid":11,"type":"string","properties":{}}},
              {"method":"publish","params":{"name":"/AIControl/MaxSpeed","pubuid":12,"type":"double","properties":{}}},
              {"method":"publish","params":{"name":"/AIControl/MaxAccel","pubuid":13,"type":"double","properties":{}}}]
        await ws.send(json.dumps(subs))
        topics={}; v={}
        t_start=None
        async def pump(dur):
            nonlocal t_start
            end=time.time()+dur
            while time.time()<end:
                try: msg=await asyncio.wait_for(ws.recv(), timeout=max(0.02,end-time.time()))
                except asyncio.TimeoutError: pass
                else:
                    if isinstance(msg,str):
                        for m in json.loads(msg):
                            if m.get("method")=="announce": topics[m["params"]["id"]]=m["params"]["name"]
                    else:
                        unp=msgpack.Unpacker(raw=False,use_list=True); unp.feed(msg)
                        for tid,ts,typ,val in unp:
                            n=topics.get(tid)
                            if n: v[n]=val
                # sample at ~20Hz
                now=time.time()
                if t_start is None: t_start=now
                if not rec.frames or now-rec.frames[-1]["t"]>=0.05:
                    pose=dbls(v.get("/AdvantageKit/RealOutputs/FieldSimulation/RobotPosition")) or v.get("/AIControl/RobotPose") or []
                    rec.frames.append(dict(
                        t=now,
                        pose=pose[:3],
                        est=v.get("/AIControl/RobotPose"),
                        target=v.get("/AIControl/ActiveTarget"),
                        traj=dbls(v.get("/AdvantageKit/RealOutputs/Odometry/Trajectory")),
                        fuel=dbls(v.get("/AdvantageKit/RealOutputs/Sim/Fuel/Positions")),
                        status=v.get("/AIControl/Status"),
                        zone=v.get("/AIControl/InShootingZone"),
                        score=v.get("/AdvantageKit/RealOutputs/Sim/Fuel/BlueScore"),
                        nav=v.get("/AIControl/Navigating"),
                        act=v.get("/AIControl/ActionRunning"),
                        rot=v.get("/AIControl/RotationLocked"),
                        action=v.get("/AIControl/LastAction"),
                        speed=v.get("/AIControl/MaxSpeed"),
                        caption=rec.caption, prompt=rec.prompt))
        us=lambda: int(time.time()*1e6)
        async def prompt(text, caption): rec.prompt=text; rec.caption=caption
        async def target(x,y,h): await ws.send(msgpack.packb([10,us(),TID["double[]"],[x,y,h]]))
        async def action(a): await ws.send(msgpack.packb([11,us(),TID["string"],a]))
        async def speed(s,a):
            await ws.send(msgpack.packb([12,us(),TID["double"],s])); await ws.send(msgpack.packb([13,us(),TID["double"],a]))

        await pump(1.5)
        # --- scripted show ---
        await prompt('"line up in the trench lane"', 'TargetPose -> [3.6, 7.4, -90]')
        await speed(3.0, 4.0); await target(3.6, 7.4, -90); await pump(4.5)

        await prompt('"run the trench and pick up fuel"', 'TargetPose + ActionTrigger INTAKE')
        await speed(3.5, 4.5); await target(9.5, 7.2, 0); await action("INTAKE"); await pump(4)
        await action("INTAKE"); await pump(3.5)

        await prompt('"shoot the fuel"', 'ActionTrigger SHOOT_FUEL  ->  out here a shot is only a pass, so it drives back into the alliance zone first')
        await action("SHOOT_FUEL"); await pump(9)
        await action("SHOOT_FUEL"); await pump(5)

        await prompt('"keep shooting while you move"', 'ActionTrigger SHOOT_ON_THE_MOVE  ->  the launch drive aims and leads while it translates')
        await action("SHOOT_ON_THE_MOVE"); await pump(6)
        await action("SHOOT_ON_THE_MOVE"); await pump(6)

        await prompt('"back off, then charge the bump"', 'MaxSpeed 4.5 m/s  ->  the bump needs a running start')
        await speed(3.5, 5.0); await target(2.2, 6.6, 0); await pump(5)
        await speed(4.5, 8.0); await target(7.5, 5.5, 0); await pump(7)

        await prompt('"stop"', 'ActionTrigger STOP')
        await action("STOP"); await pump(2.5)
    pickle.dump(rec.frames, open("/tmp/demo_frames.pkl","wb"))
    print("frames:", len(rec.frames))
    fuelmax=max((len(f["fuel"])//3 for f in rec.frames), default=0)
    print("max fuel balls in flight/on field (sim):", fuelmax)
    print("last status:", rec.frames[-1]["status"])
asyncio.run(main())
