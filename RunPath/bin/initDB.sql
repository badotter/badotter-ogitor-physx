
CREATE TABLE skeleton 
(
    id INTEGER PRIMARY KEY,
    name VARCHAR (255)
);

CREATE TABLE sequence 
(
    id INTEGER PRIMARY KEY,
    skeleton_id INTEGER,
    filename VARCHAR (255),
    name VARCHAR (255)
);

CREATE TABLE fxRigidBodyData 
(
    id INTEGER PRIMARY KEY,
    name VARCHAR (255),
    shapeFile VARCHAR(255),
    category VARCHAR (255),
    lifetime INTEGER,
    sleepThreshold FLOAT,
    shapeType INTEGER,
    dimensions_x FLOAT,
    dimensions_y FLOAT,
    dimensions_z FLOAT,
    orientation_x FLOAT,
    orientation_y FLOAT,
    orientation_z FLOAT,
    offset_x FLOAT,
    offset_y FLOAT,
    offset_z FLOAT,
    damageMultiplier FLOAT,
    isInflictor BOOLEAN,
    isKinematic BOOLEAN,
    isNoGravity BOOLEAN,
    density FLOAT,
    scale_x FLOAT,
    scale_y FLOAT,
    scale_z FLOAT
);

CREATE TABLE fxJointData 
(
    id INTEGER PRIMARY KEY,
    name VARCHAR (255),
    jointType VARCHAR (20),
    twistLimit FLOAT,
    swingLimit FLOAT,
    swingLimit2 FLOAT,
    localAxis_x FLOAT,
    localAxis_y FLOAT,
    localAxis_z FLOAT,
    localNormal_x FLOAT,
    localNormal_y FLOAT,
    localNormal_z FLOAT 
);

CREATE TABLE fxFlexBodyData 
(
    id INTEGER PRIMARY KEY,
    skeleton_id INTEGER,
    gaActionUserData_id INTEGER,
    name VARCHAR (255),
    shapeFile VARCHAR (255),
    category VARCHAR (255),
    lifetime INTEGER,
    sdk BOOLEAN,
    ga BOOLEAN,
    isKinematic BOOLEAN,
    isNoGravity BOOLEAN,
    sleepThreshold FLOAT,
    relaxType INTEGER,
    headNode VARCHAR (255),
    neckNode VARCHAR (255),
    bodyNode VARCHAR (255),
    rightFrontNode VARCHAR (255),
    leftFrontNode VARCHAR (255),
    rightBackNode VARCHAR (255),
    leftBackNode VARCHAR (255),
    tailNode VARCHAR (255),
    scale_x FLOAT,
    scale_y FLOAT,
    scale_z FLOAT
);

CREATE TABLE fxFlexBodyPartData 
(
    id INTEGER PRIMARY KEY,
    fxFlexBodyData_id INTEGER,
    fxJointData_id INTEGER,
    name VARCHAR (255),
    baseNode VARCHAR (255),
    childNode VARCHAR (255),
    shapeType VARCHAR (20),
    dimensions_x FLOAT,
    dimensions_y FLOAT,
    dimensions_z FLOAT,
    orientation_x FLOAT,
    orientation_y FLOAT,
    orientation_z FLOAT,
    offset_x FLOAT,
    offset_y FLOAT,
    offset_z FLOAT,
    damageMultiplier FLOAT,
    isInflictor BOOLEAN,
    isNoGravity BOOLEAN,
    density FLOAT    
);

CREATE TABLE persona 
(
    id INTEGER PRIMARY KEY,
    name VARCHAR (255)
    
);

CREATE TABLE personaAction
(
    id INTEGER PRIMARY KEY,
    name VARCHAR (255)
);

CREATE TABLE personaActionSequence 
(
    id INTEGER PRIMARY KEY,
    persona_id INTEGER,
    persona_action_id INTEGER,
    skeleton_id INTEGER,
    sequence_id INTEGER,
    speed FLOAT 
);

CREATE TABLE scene 
(
    id INTEGER PRIMARY KEY,
    mission_id INTEGER,
    name VARCHAR (255)
);

CREATE TABLE sceneEvent 
(
    id INTEGER PRIMARY KEY,
    actor_id INTEGER,
    scene_id INTEGER,
    type INTEGER,
    time FLOAT,
    duration FLOAT,
    node INTEGER,
    value_x FLOAT,
    value_y FLOAT,
    value_z FLOAT,
    action VARCHAR (255)
    
);

CREATE TABLE mission 
(
    id INTEGER PRIMARY KEY,
    filename VARCHAR (255)
);

CREATE TABLE gaActionUserData 
(
    id INTEGER PRIMARY KEY,
    name VARCHAR (255),
    mutationChance FLOAT,
    mutationAmount FLOAT,
    numPopulations INTEGER,
    migrateChance FLOAT,
    numRestSteps INTEGER,
    observeInterval INTEGER,
    numActionSets INTEGER,
    numSlices INTEGER,
    numSequenceReps INTEGER,
    actionName VARCHAR(255),
    fitnessData1 VARCHAR(255),
    fitnessData2 VARCHAR(255),
    fitnessData3 VARCHAR(255),
    fitnessData4 VARCHAR(255),
    fitnessData5 VARCHAR(255),
    fitnessData6 VARCHAR(255)
    
);

CREATE TABLE gaFitnessData 
(
    id INTEGER PRIMARY KEY,
    name VARCHAR (255),
    bodypartName VARCHAR(255),
    positionGoal_x FLOAT,
    positionGoal_y FLOAT,
    positionGoal_z FLOAT,
    positionGoalType_x BOOLEAN,
    positionGoalType_y BOOLEAN,
    positionGoalType_z BOOLEAN,
    rotationGoal FLOAT,
    rotationGoalType BOOLEAN
    
);


CREATE TABLE actor 
(
    id INTEGER PRIMARY KEY,
    fxFlexBodyData_id INTEGER,
    persona_id INTEGER,
    name VARCHAR (255)
);

CREATE TABLE actorScene
(
    id INTEGER PRIMARY KEY,
    actor_id INTEGER,
    scene_id INTEGER,
    playlist_id INTEGER,
    target_id INTEGER,
    start_x FLOAT,
    start_y FLOAT,
    start_z FLOAT,
    start_rot FLOAT
);

CREATE TABLE actorPlaylist
(
    id INTEGER PRIMARY KEY,
    actor_id INTEGER,
    playlist_id INTEGER,
    scene_id INTEGER
);


CREATE TABLE bvhConfig 
(
    id INTEGER PRIMARY KEY,
    name VARCHAR (255)
);

CREATE TABLE keyframeSet
(
    id INTEGER PRIMARY KEY,
    sequence_id INTEGER,
    skeleton_id INTEGER,
    actor_id INTEGER,
    scene_id INTEGER,
    name VARCHAR(255)
);

CREATE TABLE keyframe
(
    id INTEGER PRIMARY KEY,
    keyframe_set_id INTEGER,
    type INTEGER,
    time FLOAT,
    frame INTEGER,
    node INTEGER,
    value_x FLOAT,
    value_y FLOAT,
    value_z FLOAT    
);

CREATE TABLE bvhProfile
(
    id INTEGER PRIMARY KEY,
    skeleton_id INTEGER,
    name VARCHAR(255),
    scale FLOAT
);

CREATE TABLE bvhProfileNode
(
    id INTEGER PRIMARY KEY,
    bvh_profile_id INTEGER,
    bvhNodeName VARCHAR(255),
    dtsNodeName VARCHAR(255),
    nodeGroup INTEGER,
    poseRotA_x FLOAT,
    poseRotA_y FLOAT,
    poseRotA_z FLOAT,
    poseRotB_x FLOAT,
    poseRotB_y FLOAT,
    poseRotB_z FLOAT,
    fixRotA_x FLOAT,
    fixRotA_y FLOAT,
    fixRotA_z FLOAT,
    fixRotB_x FLOAT,
    fixRotB_y FLOAT,
    fixRotB_z FLOAT  
);

CREATE TABLE bvhProfileJoint
(
    id INTEGER PRIMARY KEY,
    bvh_profile_id INTEGER,
    parent_id INTEGER,
    name VARCHAR(255),
    offset_x FLOAT,
    offset_y FLOAT,
    offset_z FLOAT,
    channels INTEGER,
    channelRots_0 INTEGER,
    channelRots_1 INTEGER,
    channelRots_2 INTEGER
);

CREATE TABLE playlist
(
    id INTEGER PRIMARY KEY,
    skeleton_id INTEGER,
    name VARCHAR(255)
);

CREATE TABLE playlistSequence
(
    id INTEGER PRIMARY KEY,
    playlist_id INTEGER,
    sequence_id INTEGER,
    sequence_order FLOAT,
    repeats INTEGER,
    speed FLOAT
);

INSERT INTO persona (name) VALUES ('personaDefault');
INSERT INTO persona (name) VALUES ('personaAngry');
INSERT INTO persona (name) VALUES ('personaGoofy');
INSERT INTO persona (name) VALUES ('personaWimpy');
INSERT INTO persona (name) VALUES ('personaSleazy');

INSERT INTO skeleton (name) VALUES ('ACK');
INSERT INTO skeleton (name) VALUES ('Kork');

INSERT INTO personaAction (name) VALUES ('idle');
INSERT INTO personaAction (name) VALUES ('rSideGetup');
INSERT INTO personaAction (name) VALUES ('lSideGetup');
INSERT INTO personaAction (name) VALUES ('run');

INSERT INTO  bvhProfile (name) VALUES ('Truebones2ACK');
INSERT INTO  bvhProfile (name) VALUES ('Truebones2Kork');
