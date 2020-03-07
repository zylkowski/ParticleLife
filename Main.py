import pygame as py
import numpy as np
from ecs import *
import random as rnd
from math import *


class ParticleType:
    def __init__(self, typeName, color, chance, frictionCoefficient: float = 0.1):
        self.typeName = typeName
        self.color = color
        self.chance = chance
        self.frictionCoefficient = frictionCoefficient
        self.typeAction = []


class ParticleTypeHandler:
    types = []

    @staticmethod
    def addParticleType(particleType):
        ParticleTypeHandler.types.append(particleType)

    @staticmethod
    def getProbabilities():
        p = []
        for t in ParticleTypeHandler.types: p.append(t.chance)
        return p

    @staticmethod
    def getRandomType():
        return np.random.choice(ParticleTypeHandler.types, p=ParticleTypeHandler.getProbabilities())

    @staticmethod
    def generateActionForTypes():
        for type in ParticleTypeHandler.types:
            for typeTarget in ParticleTypeHandler.types:
                type.typeAction.append((typeTarget.typeName, (rnd.randrange(0, 2000) - 1000)))

    @staticmethod
    def getForceFor(source, target):
        for type in ParticleTypeHandler.types:
            if type.typeName == source:
                for action in type.typeAction:
                    if action[0] == target:
                        return action[1]


class Position(Component):
    def __init__(self, pos):
        self.pos = pos


class PhysicalParticle(Component):
    def __init__(self, type, initialForce, mass: float = 1):
        self.particleType = type
        self.appliedForces = initialForce
        self.mass = mass

    def applyForce(self, force):
        self.appliedForces = (self.appliedForces[0] + force[0], self.appliedForces[1] + force[1])

    def applyFriction(self):
        appliedForcesLenght = PhysicsSystem.lenght(self.appliedForces)
        frictionUnitVector = (-self.appliedForces[0] / appliedForcesLenght,
                              -self.appliedForces[1] / appliedForcesLenght)
        frictionValue = self.mass * self.particleType.frictionCoefficient

        if frictionValue > appliedForcesLenght:
            self.applyForce((-self.appliedForces[0], -self.appliedForces[1]))
        else:
            frictionVector = (frictionValue * frictionUnitVector[0], frictionValue * frictionUnitVector[1])
            self.applyForce(frictionVector)


class PhysicsSystem(System):
    def update(self, dt):
        allParticles = self.entity_manager.pairs_for_type(PhysicalParticle)
        allParticles = list(allParticles)
        self.applyAllForces(allParticles)
        self.applyFrictionForce(allParticles)

    def applyAllForces(self, allParticles):
        for particle in allParticles:
            posSource = self.entity_manager.component_for_entity(particle[0], Position).pos

            for target in allParticles:
                posTarget = self.entity_manager.component_for_entity(target[0], Position).pos
                distance = PhysicsSystem.distance(posSource, posTarget)

                if distance == 0:
                    continue
                else:
                    targetVector = (posTarget[0] - posSource[0], posTarget[1] - posSource[1])
                    len = PhysicsSystem.lenght(targetVector)
                    targetVector = (targetVector[0] / len, targetVector[1] / len)
                    sourceType = particle[1].particleType.typeName
                    targetType = target[1].particleType.typeName

                    if distance < 0.5:
                        force = 0.5 / distance
                    else:
                        force = ParticleTypeHandler.getForceFor(sourceType, targetType)

                    target[1].applyForce((force * targetVector[0], force * targetVector[1]))

    def applyFrictionForce(self, allParticles):
        for particle in allParticles:
            particle[1].applyFriction()

    @staticmethod
    def lenght(v):
        return sqrt(v[0] ** 2 + v[1] ** 2)

    @staticmethod
    def distance(a, b):
        return sqrt(abs(a[0] - b[0]) ** 2 + abs(a[1] - b[1]) ** 2)


class Simulation:
    def __init__(self, numberOfParticles: int = 100, sceneSize: (int, int) = (100, 100)):
        self.entityManager = EntityManager()
        self.systemManager = SystemManager(self.entityManager)
        self.physicsSystem = PhysicsSystem()
        self.systemManager.add_system(self.physicsSystem)

        self.numberOfParticles = numberOfParticles
        self.sceneSize = sceneSize
        self.particles = []

        for i in range(0, numberOfParticles): self.createParticle()

    def createParticle(self):
        entity = self.entityManager.create_entity()

        pos = (rnd.randrange(0, self.sceneSize[0]), rnd.randrange(0, self.sceneSize[1]))
        self.entityManager.add_component(entity, Position(pos))

        initialForce = (rnd.randrange(0, 3), rnd.randrange(0, 3))
        type = ParticleTypeHandler.getRandomType()
        self.entityManager.add_component(entity, PhysicalParticle(type, initialForce))

        self.particles.append(entity)

    def printParticlesWithInfo(self):
        for p in self.particles:
            pos = self.entityManager.component_for_entity(p, Position).pos
            physicsComponent = self.entityManager.component_for_entity(p, PhysicalParticle)
            print(
                f"particle type: {physicsComponent.particleType.typeName}\n\t\t\t pos: {pos}\n\t\t\t appliedForces: {physicsComponent.appliedForces}")

    @staticmethod
    def createParticleTypes():
        ParticleTypeHandler.addParticleType(ParticleType("Green", py.Color(0, 255, 0), 0.3))
        ParticleTypeHandler.addParticleType(ParticleType("Red", py.Color(255, 0, 0), 0.3))
        ParticleTypeHandler.addParticleType(ParticleType("Blue", py.Color(0, 0, 255), 0.2))
        ParticleTypeHandler.addParticleType(ParticleType("Yellow", py.Color(255, 255, 0), 0.2))

        ParticleTypeHandler.generateActionForTypes()

    @staticmethod
    def printParticleTypes():
        for type in ParticleTypeHandler.types:
            print(
                f"type: {type.typeName}\n\t\t frictionCoefficient: {type.frictionCoefficient}\n\t\t actions: {type.typeAction}")


if __name__ == "__main__":
    Simulation.createParticleTypes()
    Simulation.printParticleTypes()

    sim = Simulation(numberOfParticles=100, sceneSize=(200, 200))

    # sim.printParticlesWithInfo()
    sim.systemManager.update(1)
