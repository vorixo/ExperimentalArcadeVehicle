// Fill out your copyright notice in the Description page of Project Settings.


#include "AVBaseVehicle.h"
#include "CollisionQueryParams.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "SuspensionUtility.h"
#include "Physics/Experimental/PhysScene_Chaos.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"
#include "Chaos/Utilities.h"
#include "Chaos/Particle/ObjectState.h"

#if WITH_EDITOR
#include "PhysicsEngine/PhysicsSettings.h"
#endif

static int32 bDebugInfo = 0;
FAutoConsoleVariableRef CVARDebugPlayableArea(
	TEXT("AV.DisplayDebugInfo"),
	bDebugInfo,
	TEXT("Draws in 3D space the vehicle information"),
	ECVF_Cheat);

// Sets default values
AAVBaseVehicle::AAVBaseVehicle()
{
 	// Set this pawn to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;
	PrimaryActorTick.bStartWithTickEnabled = false;
	PrimaryActorTick.bAllowTickOnDedicatedServer = false;

	// Custom Properties
	SuspensionFront = FSuspensionData();
	SuspensionRear = FSuspensionData();
	bIsMovingOnGround = false;
	bIsCloseToGround = false;
	GravityAir = -2000.f;
	GravityGround = -2000.f;
	
	// Gameplay driven friction
	LateralFrictionModifier = 1.f;
	bStickyWheels = false;
	// Default to z grav
	AvgedNormals = FVector::UpVector;
	LinearDampingAir = 0.01f;
	AngularDamping = 10.0f;
	MaxSpeedBoosting = 3000.f;
	CurrentAngularSpeed = 0.f;
	CachedSpeedSized = 0.f;
	// Maximum speed that can ever be achieved, accelerating or not.
	TerminalSpeed = 3500.f;
	CurrentThrottleAxis = 0.f;
	CurrentBrakeAxis = 0.f;
	bTiltedThrottle = true;
	VehicleAcceleration = 5000.f;
	VehicleBoostAcceleration = 5000.f;
	AirTorqueAcceleration = 15.f;
	TorqueAcceleration = 12.f;
	AirStrafeAcceleration = 1000.f;
	BrakinDeceleration = 3000.f;
	GroundDetectionDistanceThreshold = 800.f;
	AccelerationCenterOfMassOffset = FVector2D(50.f, 40.f);
	RGUpVector = FVector::ZeroVector;
	RGRightVector = FVector::ZeroVector;
	RGForwardVector = FVector::ZeroVector;
	CurrentGroundFriction = DEFAULT_GROUND_FRICTION;
	CurrentGroundScalarResistance = DEFAULT_GROUND_RESISTANCE;
	LegalSpeedOffset = 100.f;
	bCompletelyInTheAir = false;
	bCompletelyInTheGround = true;
	bOrientRotationToMovementInAir = true;
	OrientRotationToMovementInAirInfluenceRate = 1.f;
	AccelerationInfluenceRateWhileBraking = 0.3f;
	SteeringDecelerationSideVelocityFactorScale = 40.f;
	SteeringDecelerationSideVelocityInterpolationSpeed = 0.4;
	SwapGearDirectionDelay = 0.4f;
	StopThreshold = 40.f;
	InfluencialDirection = 0.f;
	bIsGearReady = true;
	AirNavigationMode = EAirNavigationMode::None;
	AirNavigationAdaptativeForce = 25.f;
	LastUsedGear = ESimplifiedDirection::Idle;
	LastUsedGroundGear = ESimplifiedDirection::Idle;
	bFlipping = false;
	bComputeAxleForces = false;
	bComputePhysicsSimulatedProxy = true;
	ResetVehicle();

	CollisionMesh = CreateDefaultSubobject<UStaticMeshComponent>("CollisionMesh");
	if (CollisionMesh)
	{
		CollisionMesh->SetSimulatePhysics(true);
		CollisionMesh->BodyInstance.bOverrideMass = true;
		// Important: The physics setup is modelled to work with vehicles with mass surrounding 1.3 KG
		CollisionMesh->BodyInstance.SetMassOverride(1.3f);
		CollisionMesh->SetEnableGravity(false);
		CollisionMesh->bReplicatePhysicsToAutonomousProxy = false;
		CollisionMesh->BodyInstance.COMNudge = FVector(0.f, 0.f, -20.f);
		CollisionMesh->SetCollisionProfileName(UCollisionProfile::Vehicle_ProfileName);
		RootComponent = CollisionMesh;
	}
	
	// Component instanciation
	BackRight = FVector(-75.f, 50.f, -25.f);
	FrontRight = FVector(75.f, 50.f, -25.f);
	FrontLeft = FVector(75.f, -50.f, -25.f);
	BackLeft = FVector(-75.f, -50.f, -25.f);

#if WITH_EDITOR
	bHideHelpHandlersInPIE = true;

	BackRightHandle = CreateDefaultSubobject<UStaticMeshComponent>("BackRightHandle");
	FrontRightHandle = CreateDefaultSubobject<UStaticMeshComponent>("FrontRightHandle");
	FrontLeftHandle = CreateDefaultSubobject<UStaticMeshComponent>("FrontLeftHandle");
	BackLeftHandle = CreateDefaultSubobject<UStaticMeshComponent>("BackLeftHandle");
	if (BackLeftHandle && BackRightHandle && FrontRightHandle && FrontLeftHandle)
	{
		static ConstructorHelpers::FObjectFinder<UStaticMesh> CylinderHelpMesh(TEXT("StaticMesh'/Engine/BasicShapes/Cube1.Cube1'"));
		BackRightHandle->SetStaticMesh(CylinderHelpMesh.Object);
		FrontRightHandle->SetStaticMesh(CylinderHelpMesh.Object);
		FrontLeftHandle->SetStaticMesh(CylinderHelpMesh.Object);
		BackLeftHandle->SetStaticMesh(CylinderHelpMesh.Object);
		
		BackRightHandle->SetupAttachment(CollisionMesh);
		FrontRightHandle->SetupAttachment(CollisionMesh);
		FrontLeftHandle->SetupAttachment(CollisionMesh);
		BackLeftHandle->SetupAttachment(CollisionMesh);
		
		BackRightHandle->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		FrontRightHandle->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		FrontLeftHandle->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		BackLeftHandle->SetCollisionEnabled(ECollisionEnabled::NoCollision);

		UpdateHandlersTransformCDO();
	}
#endif //WITH_EDITOR

}

/** Util for drawing result of single box trace  */
void DrawDebugSweptBox(const UWorld* InWorld, FVector const& Start, FVector const& End, FQuat const& CapsuleRot, FVector const& HalfSize, EDrawDebugTrace::Type DrawDebugType, FColor const& Color, bool bPersistentLines, float LifeTime)
{
	if (DrawDebugType != EDrawDebugTrace::None)
	{
		FVector const TraceVec = End - Start;

		::DrawDebugBox(InWorld, Start, HalfSize, CapsuleRot, Color, bPersistentLines, LifeTime, 0);

		//now draw lines from vertices
		FVector Vertices[8];
		Vertices[0] = Start + CapsuleRot.RotateVector(FVector(-HalfSize.X, -HalfSize.Y, -HalfSize.Z));	//flt
		Vertices[1] = Start + CapsuleRot.RotateVector(FVector(-HalfSize.X, HalfSize.Y, -HalfSize.Z));	//frt
		Vertices[2] = Start + CapsuleRot.RotateVector(FVector(-HalfSize.X, -HalfSize.Y, HalfSize.Z));	//flb
		Vertices[3] = Start + CapsuleRot.RotateVector(FVector(-HalfSize.X, HalfSize.Y, HalfSize.Z));	//frb
		Vertices[4] = Start + CapsuleRot.RotateVector(FVector(HalfSize.X, -HalfSize.Y, -HalfSize.Z));	//blt
		Vertices[5] = Start + CapsuleRot.RotateVector(FVector(HalfSize.X, HalfSize.Y, -HalfSize.Z));	//brt
		Vertices[6] = Start + CapsuleRot.RotateVector(FVector(HalfSize.X, -HalfSize.Y, HalfSize.Z));	//blb
		Vertices[7] = Start + CapsuleRot.RotateVector(FVector(HalfSize.X, HalfSize.Y, HalfSize.Z));		//brb
		for (int32 VertexIdx = 0; VertexIdx < 8; ++VertexIdx)
		{
			::DrawDebugLine(InWorld, Vertices[VertexIdx], Vertices[VertexIdx] + TraceVec, Color, bPersistentLines, LifeTime, 0);
		}

		::DrawDebugBox(InWorld, End, HalfSize, CapsuleRot, Color, bPersistentLines, LifeTime, 0);
	}
}


// Called when the game starts or when spawned
void AAVBaseVehicle::BeginPlay()
{
	const UPrimitiveComponent* PawnRootComponent = Cast<UPrimitiveComponent>(GetRootComponent());
	
	if (PawnRootComponent != NULL)
	{
		RootBodyInstance = PawnRootComponent->GetBodyInstance();
		UWorld* World = GetWorld();
		if (World->IsGameWorld() && (IsLocallyControlled() || bComputePhysicsSimulatedProxy))
		{
			FPhysScene* PScene = World->GetPhysicsScene();
			if (PScene)
			{
				// Register physics step delegate
				PScene->RegisterAsyncPhysicsTickActor(this);
				InitVehicle();
				// Very basic sanity check to ensure someone is not adding more wheels than the absolutely necesary
				ensure(NUMBER_OF_WHEELS == CachedSuspensionInfo.Num());
			}
		}
	}

	Super::BeginPlay();
}

void AddForceAtPosition(Chaos::FRigidBodyHandle_Internal* RigidHandle, FVector Force, FVector Position)
{
	if (ensure(RigidHandle))
	{
		const Chaos::FVec3 WorldCOM = Chaos::FParticleUtilitiesGT::GetCoMWorldPosition(RigidHandle);
		const Chaos::FVec3 WorldTorque = Chaos::FVec3::CrossProduct(Position - WorldCOM, Force);
		RigidHandle->AddForce(Force, false);
		RigidHandle->AddTorque(WorldTorque, false);
	}
}

void AAVBaseVehicle::PhysicsTick(float SubstepDeltaTime)
{
	if (!RootBodyInstance) return;

#if WITH_EDITOR
	const UPhysicsSettings* Settings = UPhysicsSettings::Get();
	if (Settings && GetWorld()->GetTimeSeconds() > 5.f)
	{
		const float RequiredSteps = GetWorld()->GetDeltaSeconds() / Settings->MaxSubstepDeltaTime;
		if (bDebugInfo && !ensure(RequiredSteps <= Settings->MaxSubsteps))
		{
			const FString SimulationMessage = FString::Printf(TEXT("The simulation requires more steps to be representative at the current Delta Time. Consider increasing MaxSubsteps if you desire MaxSubstepDeltaTime precision. Required Steps: %d"), FMath::CeilToInt(RequiredSteps));
			PRINT_TICK(SimulationMessage);
		}
	}
#endif

	if (const auto Handle = RootBodyInstance->ActorHandle)
	{
		RigidHandle = Handle->GetPhysicsThreadAPI();
	}

	if (!RigidHandle)
	{
		return;
	}

	// Getting the transformation matrix of the object
	RGWorldTransform = FTransform(RigidHandle->R(), RigidHandle->X());

	// World Location
	RGLocation = RGWorldTransform.GetLocation();

	// Getting the forward, right, and up vectors
	RGForwardVector = RGWorldTransform.GetUnitAxis(EAxis::X);
	RGRightVector = RGWorldTransform.GetUnitAxis(EAxis::Y);
	RGUpVector = RGWorldTransform.GetUnitAxis(EAxis::Z);

	// Calc velocities and Speed
	CurrentHorizontalVelocity = RigidHandle->V();
	CachedSpeedSized = CurrentHorizontalVelocity.Size();
	LocalVelocity = RGWorldTransform.InverseTransformVectorNoScale(CurrentHorizontalVelocity);
	CurrentAngularVelocity = RigidHandle->W();
	CurrentAngularSpeed = FVector::DotProduct(CurrentAngularVelocity, RGUpVector);

	// Reset runtime particle state
	RigidHandle->SetObjectState(Chaos::EObjectStateType::Dynamic, true);

	// Moving on top of moving surfaces
	ComputeBasedMovement();

	// Input processing
	ApplyInputStack(SubstepDeltaTime);

	// Suspension and friction forces
	ApplySuspensionForces(SubstepDeltaTime);

	// Simulate wheel animation 
	SimulateWheelMovement(SubstepDeltaTime);

	/************************************************************************/
	/* Apply gravity and sliding forces                                     */
	/************************************************************************/
	ApplyGravityForce(SubstepDeltaTime);

	if (bIsMovingOnGround)
	{
		// Drag/Friction force
		RigidHandle->AddForce(GetLateralFrictionDragForce(), false);
	}
	
	/************************************************************************/
	/* End apply gravity and sliding forces									*/
	/************************************************************************/


	/************************************************************************/
	/* Apply deceleration forces			                                */
	/************************************************************************/
	const ESimplifiedDirection CurrentSimplifiedDirection = GetSimplifiedKartDirection();
	if (CurrentSimplifiedDirection == ESimplifiedDirection::Idle && bIsGearReady)
	{
		// Compute desired direction based on axis action
		if ((LastUsedGear == ESimplifiedDirection::Forward && InfluencialDirection < -0.1f) || 
			(LastUsedGear == ESimplifiedDirection::Reverse && InfluencialDirection > 0.1f))
		{
			LastTimeGearSwapped = GetWorld()->GetTimeSeconds();
		}
	}

	const bool bWantsToIdle = (CurrentSimplifiedDirection == ESimplifiedDirection::Idle && FMath::IsNearlyZero(InfluencialDirection) && !bIsBoosting);
		
	if (bWantsToIdle)
	{
		AccelerationAccumulatedTime = 0.f;
		DecelerationAccumulatedTime = MaxDecelerationCurveTime;
		ThrottleForce = FVector::ZeroVector;
	}
	else
	{
		// START - Substract side velocity scaled from forward velocity  when turning
		const FVector side_velocity = FMath::Abs(LocalVelocity.Y) * RGForwardVector;
		const FVector non_forward_velocity = CurrentHorizontalVelocity.GetSafeNormal2D() * getMaxSpeed() - (side_velocity * SteeringDecelerationSideVelocityFactorScale);
		const bool bTurnDeceleration = (LocalVelocity.X * LocalVelocity.X) > non_forward_velocity.SizeSquared() && !bIsBoosting && 
			!FMath::IsNearlyZero(CurrentSteeringAxis) && CurrentSimplifiedDirection != ESimplifiedDirection::Idle;

		if (bTurnDeceleration)
		{
			ThrottleForce = (non_forward_velocity.GetSafeNormal2D() * -(LocalVelocity.X) * SteeringDecelerationSideVelocityInterpolationSpeed);
		}
		// END - Substract side velocity scaled from forward velocity  when turning


		const float SpeedRatio = CachedSpeedSized / GetAbsMaxSpeedAxisIndependent();
		const bool bNoInput = (CurrentBrakeAxis > -0.1 && CurrentThrottleAxis < 0.1 && !bIsBoosting);
		
		// Clamping curve thresholds
		const float AccumulativeAxisAccel = FMath::Clamp(AccelerationAccumulatedTime, MinAccelerationCurveTime, MaxAccelerationCurveTime);
		const float AccumulativeAxisDecel = DecelerationAccumulatedTime > MaxDecelerationCurveTime ? MaxDecelerationCurveTime : DecelerationAccumulatedTime + SubstepDeltaTime;
		
		// Acceleration and deceleration mappings
		AccelerationAccumulatedTime = (bNoInput || bTurnDeceleration || IsBraking()) ? 
			FMath::GetMappedRangeValueClamped(FVector2D(-1, 0), FVector2D(MinAccelerationCurveTime, 0.f), SpeedRatio) + 
			FMath::GetMappedRangeValueClamped(FVector2D(0, 1), FVector2D(0.f, MaxAccelerationCurveTime), SpeedRatio) :
			AccumulativeAxisAccel;
		DecelerationAccumulatedTime = (bNoInput) ? AccumulativeAxisDecel : FMath::GetMappedRangeValueClamped(FVector2D(0, 1), FVector2D(MaxDecelerationCurveTime, 0), FMath::Abs(SpeedRatio));

		// Engine deceleration only if no input is applied whatsoever
		const FVector ThrottleAdjustmentRatio = CurrentHorizontalVelocity * (bNoInput) * GetDecelerationRatio();

		// Forcing vehicle decceleration if they surpass the LegalSpeedOffset
		const bool bIsGoingOverLegalSpeed = (CachedSpeedSized > ((GetAbsMaxSpeedAxisIndependent() * CurrentGroundScalarResistance) + LegalSpeedOffset));
		ThrottleForce = bIsGoingOverLegalSpeed ? FVector::VectorPlaneProject(-CurrentHorizontalVelocity.GetSafeNormal(), AvgedNormals) * getAcceleration() : ThrottleForce;

		// Adjusting Throttle based on engine decceleration values
		ThrottleForce = bIsMovingOnGround ? ThrottleForce - ThrottleAdjustmentRatio : ThrottleForce;
	}

	// Forward and backwards speed work with 0 damping.
	RigidHandle->SetLinearEtherDrag(bIsMovingOnGround ? 0.f : LinearDampingAir);
	RigidHandle->SetAngularEtherDrag(AngularDamping);


	/************************************************************************/
	/* Apply movement forces                                                */
	/************************************************************************/
	
	RigidHandle->AddTorque(Chaos::Utilities::ComputeWorldSpaceInertia(RigidHandle->R() * RigidHandle->RotationOfMass(), RigidHandle->I()) * SteeringForce, false);

	
	if (bTiltedThrottle)
	{
		AddForceAtPosition(RigidHandle, ThrottleForce, GetOffsetedCenterOfVehicle());
	}
	else
	{
		RigidHandle->AddForce(ThrottleForce, false);
	}
	
	// Vehicles can't go beyond the terminal speed. This code snippet also smooths out current velocity towards max speed leaving some room for physical enviro impulses
	if (CachedSpeedSized > FMath::Min(GetAbsMaxSpeedAxisIndependent(), GetTerminalSpeed()))
	{
		const float BlendAlpha = (CachedSpeedSized - GetAbsMaxSpeedAxisIndependent()) / (GetTerminalSpeed() - GetAbsMaxSpeedAxisIndependent());
		const float TerminalVelocityPreemptionFinalForce = CachedSpeedSized > GetTerminalSpeed() ? BIG_NUMBER : FMath::Lerp(0.f, GetTerminalSpeed() + TERMINAL_VELOCITY_PREEMPTION_FORCE_OFFSET, BlendAlpha);
		RigidHandle->AddForce(-CurrentHorizontalVelocity.GetSafeNormal() * TerminalVelocityPreemptionFinalForce, false);
	}
	
	// We also want to help the vehicle get into an idle state
	if (CachedSpeedSized <= StopThreshold)
	{
		const float BlendAlpha = CachedSpeedSized/(StopThreshold);
		const float TerminalVelocityPreemptionFinalForce = FMath::Lerp(0.f, IDLE_VEHICLE_FORCE, BlendAlpha);
		RigidHandle->AddForce(-CurrentHorizontalVelocity.GetSafeNormal() * TerminalVelocityPreemptionFinalForce, false);
	}
	
	/************************************************************************/
	/* End apply movement forces                                            */
	/************************************************************************/

	if (bDebugInfo > 0)
	{
		UKismetSystemLibrary::DrawDebugSphere(this, GetOffsetedCenterOfVehicle(), 10.f, 12);
		UKismetSystemLibrary::DrawDebugSphere(this, RGLocation, 10.f, 12);
		UKismetSystemLibrary::DrawDebugString(this, RGLocation, FString::SanitizeFloat(LocalVelocity.X).Mid(0, 5), NULL,FLinearColor::Red);
	}

	// TODO: Vehicle Effects

	// Rest variables - Last tick calculations
	LastUsedGear = CurrentSimplifiedDirection;
	LastUsedGroundGear = bIsMovingOnGround ? LastUsedGear : LastUsedGroundGear;
	ThrottleForce = FVector::ZeroVector;
	SteeringForce = FVector::ZeroVector;
}

float AAVBaseVehicle::GetSuspensionForceMagnitude(const FCachedSuspensionInfo& InCachedInfo, float DeltaTime) const
{									
	const float LocalWheelVelocity = (InCachedInfo.DisplacementInput - InCachedInfo.LastDisplacement)/DeltaTime;

	const float SpringDisplacement = InCachedInfo.SuspensionLength - InCachedInfo.DisplacementInput;

	const float Damping = (InCachedInfo.DisplacementInput < InCachedInfo.LastDisplacement) ? InCachedInfo.BoundDamping : InCachedInfo.ReboundDamping;
	const float StiffnessForce = SpringDisplacement * InCachedInfo.SuspensionData.SpringRate;
	const float DampingForce = LocalWheelVelocity * Damping;
	const float SuspensionForce = StiffnessForce - DampingForce;

	return SuspensionForce;
}


FVector AAVBaseVehicle::GetLateralFrictionDragForce() const
{
	const FVector FrictionDragForce = ((FVector::DotProduct(RGRightVector, CurrentHorizontalVelocity) * -1.f) * (LateralFrictionModifier * CurrentGroundFriction * BASE_GROUND_FRICTION)) * RGRightVector;
	return FrictionDragForce;
}


void AAVBaseVehicle::ApplySuspensionForces(float DeltaTime)
{
	// Compute suspension forces
	TArray<FSuspensionHitInfo> HitInfos;
	HitInfos.Init(FSuspensionHitInfo(), NUMBER_OF_WHEELS);

	for (int SpringIdx = 0; SpringIdx < NUMBER_OF_WHEELS; SpringIdx++)
	{
		auto& Susp = CachedSuspensionInfo[SpringIdx];

		const FVector WheelRestingWorldLocation = RGWorldTransform.TransformPosition(Susp.WheelRelativeLocation);
		const FVector TraceStart = WheelRestingWorldLocation + RGUpVector * Susp.SuspensionData.SuspensionMaxRaise;
		const FVector TraceEnd = WheelRestingWorldLocation - RGUpVector * (GroundDetectionDistanceThreshold);
		const float TraceFullLength = Susp.SuspensionLength + Susp.SuspensionData.WheelRadius;

		auto& sHitInfo = HitInfos[SpringIdx];

		sHitInfo.WheelRestingWorldLocation = WheelRestingWorldLocation;
		Susp.DisplacementInput = TraceFullLength - Susp.SuspensionData.WheelRadius;
		Susp.bWheelOnGround = false;
		Susp.PhysMat = NULL;

		FHitResult OutHit;
		if (TraceFunc(TraceStart, TraceEnd, Susp.SuspensionData.TraceHalfSize, bDebugInfo > 0 ? EDrawDebugTrace::ForOneFrame : EDrawDebugTrace::None, OutHit))
		{
			#if ENABLE_DRAW_DEBUG
			if (bDebugInfo)
			{
				UWorld* World = GEngine->GetWorldFromContextObject(this, EGetWorldErrorMode::LogAndReturnNull);
				if (World)
				{
					UKismetSystemLibrary::DrawDebugString(World, TraceStart, FString::SanitizeFloat(Susp.LastDisplacement).Mid(0, 4), nullptr, FLinearColor::White, 0.f);
					DrawDebugSweptBox(GetWorld(), TraceStart, TraceStart - (RGUpVector * TraceFullLength), RGWorldTransform.GetRotation(),
						FVector(Susp.SuspensionData.TraceHalfSize.X, Susp.SuspensionData.TraceHalfSize.Y, 0.f), EDrawDebugTrace::ForOneFrame, Susp.LastDisplacement > 0.f ? FColor::Blue : FColor::Red, false, 0.f);

					const FVector WheelWorldLocation = TraceStart - (RGUpVector * Susp.LastDisplacement);

					UKismetSystemLibrary::DrawDebugCylinder(this, WheelWorldLocation + (RGRightVector * (Susp.SuspensionData.TraceHalfSize.X/2)), WheelWorldLocation - (RGRightVector * (Susp.SuspensionData.TraceHalfSize.X/2)),
						Susp.SuspensionData.WheelRadius, 12, FLinearColor::White, 0.f);

					UKismetSystemLibrary::DrawDebugCylinder(this, WheelRestingWorldLocation + (RGRightVector * (Susp.SuspensionData.TraceHalfSize.X/2)), WheelRestingWorldLocation - (RGRightVector * (Susp.SuspensionData.TraceHalfSize.X/2)),
						Susp.SuspensionData.WheelRadius, 12, FLinearColor::Black, 0.f);
				}
			}
			#endif

			Susp.bWheelOnGround = (OutHit.Distance <= TraceFullLength);
			
			if (Susp.bWheelOnGround)
			{
				Susp.DisplacementInput = OutHit.Distance - Susp.SuspensionData.WheelRadius;
				const float ForceMagnitude = GetSuspensionForceMagnitude(Susp, DeltaTime);

				const FVector FinalForce = (OutHit.ImpactNormal * ForceMagnitude);

				AddForceAtPosition(RigidHandle, FinalForce, WheelRestingWorldLocation);
				
				// Storing resting force for axle calculus later
				sHitInfo.SusForce = Susp.SuspensionData.SuspensionLoadRatio * ForceMagnitude + (1.f - Susp.SuspensionData.SuspensionLoadRatio) * Susp.RestingForce;

				// Physical material properties for this wheel contacting the ground
				const bool bPhysMatExists = OutHit.PhysMaterial.IsValid();
				Susp.PhysMat = OutHit.PhysMaterial;
				sHitInfo.GroundFriction = bPhysMatExists ? OutHit.PhysMaterial->Friction : DEFAULT_GROUND_FRICTION;
				sHitInfo.GroundResistance = bPhysMatExists ? OutHit.PhysMaterial->Density : DEFAULT_GROUND_RESISTANCE;
			}
			sHitInfo.bTraceHit = true;
			Susp.ImpactNormal = OutHit.ImpactNormal;
		}
		Susp.LastDisplacement = Susp.DisplacementInput;
	}
	
	// Compute axle load balancing forces
	if (bComputeAxleForces)
	{
		for (int AxleIdx = 0; AxleIdx < NUMBER_OF_AXLES; AxleIdx++)
		{
			const int WheelIdxA = AxleIdx * NUMBER_OF_AXLES;
			const int WheelIdxB = WheelIdxA + 1;

			const float RollbarScaling = 1.f;
			const float ForceDiffOnAxleF = HitInfos[WheelIdxA].SusForce - HitInfos[WheelIdxB].SusForce;
			const FVector ForceVector0 = RGUpVector * ForceDiffOnAxleF * RollbarScaling;
			const FVector ForceVector1 = RGUpVector * ForceDiffOnAxleF * -RollbarScaling;

			AddForceAtPosition(RigidHandle, ForceVector0, HitInfos[WheelIdxA].WheelRestingWorldLocation);
			AddForceAtPosition(RigidHandle, ForceVector1, HitInfos[WheelIdxB].WheelRestingWorldLocation);
		}
	}

	// On landing impl. If in the previous step it was in the air and now is in the ground 
	const bool bWasMovingOnGround = bIsMovingOnGround;

	const uint8 WheelsTouchingTheGround = (CachedSuspensionInfo[FRONT_LEFT].bWheelOnGround + CachedSuspensionInfo[FRONT_RIGHT].bWheelOnGround + CachedSuspensionInfo[BACK_LEFT].bWheelOnGround + CachedSuspensionInfo[BACK_RIGHT].bWheelOnGround);
	bIsMovingOnGround = WheelsTouchingTheGround > 2;
	bCompletelyInTheAir = WheelsTouchingTheGround == 0;
	bCompletelyInTheGround = WheelsTouchingTheGround == NUMBER_OF_WHEELS;
	bIsCloseToGround = (HitInfos[FRONT_LEFT].bTraceHit + HitInfos[FRONT_RIGHT].bTraceHit + HitInfos[BACK_LEFT].bTraceHit + HitInfos[BACK_RIGHT].bTraceHit) > 2;
	CurrentGroundFriction = (HitInfos[FRONT_LEFT].GroundFriction + HitInfos[FRONT_RIGHT].GroundFriction + HitInfos[BACK_LEFT].GroundFriction + HitInfos[BACK_RIGHT].GroundFriction) / NUMBER_OF_WHEELS;
	CurrentGroundScalarResistance = (HitInfos[FRONT_LEFT].GroundResistance + HitInfos[FRONT_RIGHT].GroundResistance + HitInfos[BACK_LEFT].GroundResistance + HitInfos[BACK_RIGHT].GroundResistance) / NUMBER_OF_WHEELS;
	AvgedNormals = (CachedSuspensionInfo[FRONT_LEFT].ImpactNormal + CachedSuspensionInfo[FRONT_RIGHT].ImpactNormal + CachedSuspensionInfo[BACK_LEFT].ImpactNormal + CachedSuspensionInfo[BACK_RIGHT].ImpactNormal) / NUMBER_OF_WHEELS;

	// On landed event
	if (!bWasMovingOnGround && bIsMovingOnGround)
	{
		Landed(AvgedNormals);
	}
}


void AAVBaseVehicle::SimulateWheelMovement(float DeltaTime)
{
	// Front wheels
	const float GroundOmegaX = LocalVelocity.X / CachedSuspensionInfo[FRONT_LEFT].SuspensionData.WheelRadius;
	WheelAnimData.WheelSpinningSpeed.X += (GroundOmegaX - WheelAnimData.WheelSpinningSpeed.X);
	WheelAnimData.WheelAngularPosition.X -= WheelAnimData.WheelSpinningSpeed.X * DeltaTime;
	WheelAnimData.WheelCurrentSteer.X = FMath::FInterpTo(WheelAnimData.WheelCurrentSteer.X, CurrentSteeringAxis * WheelAnimData.WheelMaxSteerAngle.X, DeltaTime, 10.f);
	// Rear wheels
	const float GroundOmegaY = LocalVelocity.X / CachedSuspensionInfo[BACK_LEFT].SuspensionData.WheelRadius;
	WheelAnimData.WheelSpinningSpeed.Y += (GroundOmegaY - WheelAnimData.WheelSpinningSpeed.Y);
	WheelAnimData.WheelAngularPosition.Y -= WheelAnimData.WheelSpinningSpeed.Y * DeltaTime;
	WheelAnimData.WheelCurrentSteer.Y = FMath::FInterpTo(WheelAnimData.WheelCurrentSteer.Y, -CurrentSteeringAxis * WheelAnimData.WheelMaxSteerAngle.Y, DeltaTime, 10.f);
}


void AAVBaseVehicle::Landed(const FVector& HitNormal)
{
	TimeFalling = 0.f;
	OnLanded(AvgedNormals);
}


void AAVBaseVehicle::ComputeBasedMovement()
{
	if (BasedMovementSetup == EBasedPlatformSetup::IgnoreBasedMovement)
	{
		return;
	}

	// Monolithic function approach
	FHitResult BasedHit(ForceInit);
	static const FName BasedLineTraceName(TEXT("BasedTrace"));
	FCollisionQueryParams TraceParams;
	TraceParams.TraceTag = BasedLineTraceName;
	TraceParams.bTraceComplex = false;
	TraceParams.bReturnPhysicalMaterial = false;
	TraceParams.AddIgnoredActor(this);
	bool const bHit = GetWorld()->LineTraceSingleByChannel(BasedHit, RGLocation, RGLocation + (-RGUpVector * 500.f), ECC_Visibility, TraceParams);
	
	// New base acquired
	if (bHit && BasedPlatformInfo.MovementBase != BasedHit.GetComponent())
	{
		BasedPlatformInfo.MovementBase = BasedHit.GetComponent();
		BasedPlatformInfo.Location = BasedPlatformInfo.MovementBase->GetComponentLocation();
		BasedPlatformInfo.Rotation = BasedPlatformInfo.MovementBase->GetComponentQuat();
	}
	else if (BasedPlatformInfo.MovementBase)
	{
		const FVector NewBasedLocation = BasedPlatformInfo.MovementBase->GetComponentLocation();
		const FQuat NewBasedQuat = BasedPlatformInfo.MovementBase->GetComponentQuat();
		const bool bApplyBodyTransform = bIsMovingOnGround && (!BasedPlatformInfo.Rotation.Equals(NewBasedQuat, 1e-8f) || NewBasedLocation != BasedPlatformInfo.Location);

		if (bApplyBodyTransform)
		{	
			FTransform NewTransform = RGWorldTransform;	

			// Moving platforms
			if (BasedMovementSetup != EBasedPlatformSetup::IgnoreBasedRotation)
			{
				// Finding the change in rotation
				const FQuat DeltaQuat = NewBasedQuat * BasedPlatformInfo.Rotation.Inverse();
				const FQuat TargetQuat = DeltaQuat * RGWorldTransform.GetRotation();
				NewTransform.SetRotation(TargetQuat);
			}
			
			// A rotation might have location offset (long objects rotating)
			const FQuatRotationTranslationMatrix OldLocalToWorld(BasedPlatformInfo.Rotation, BasedPlatformInfo.Location);
			const FQuatRotationTranslationMatrix NewLocalToWorld(NewBasedQuat, NewBasedLocation);
			FVector const LocalBasePos = OldLocalToWorld.InverseTransformPosition(RGLocation);
			FVector const NewWorldPos = NewLocalToWorld.TransformPosition(LocalBasePos);
			FVector const DeltaPosition = NewWorldPos - RGLocation;
			NewTransform.AddToTranslation(DeltaPosition);

			RigidHandle->SetX(NewTransform.GetLocation(), false);
			RigidHandle->SetR(NewTransform.GetRotation(), false);
		}
		
		BasedPlatformInfo.Location = NewBasedLocation;
		BasedPlatformInfo.Rotation = NewBasedQuat;
	}
}


bool AAVBaseVehicle::TraceFunc_Implementation(FVector Start, FVector End, FVector2D HalfSize, EDrawDebugTrace::Type DrawDebugType, FHitResult& OutHit)
 {
	FHitResult Hit(ForceInit);
	static const FName HoverLineTraceName(TEXT("HoverTrace"));
	FCollisionQueryParams TraceParams;
	TraceParams.TraceTag = HoverLineTraceName;
	TraceParams.bTraceComplex = false;
	TraceParams.bReturnPhysicalMaterial = true;
	TraceParams.AddIgnoredActor(this);
	bool const bHit = GetWorld()->SweepSingleByChannel(OutHit, Start, End, RGWorldTransform.GetRotation(), ECC_Visibility, FCollisionShape::MakeBox(FVector(HalfSize.X, HalfSize.Y, 0.f)), TraceParams);
#if ENABLE_DRAW_DEBUG
	DrawDebugSweptBox(GetWorld(), Start, End, RGWorldTransform.GetRotation(), FVector(HalfSize.X, HalfSize.Y, 0.f), DrawDebugType, bHit ? FColor::Green : FColor::Red, false, 0.f);
#endif
	return bHit;
}


FVector AAVBaseVehicle::GetOffsetedCenterOfVehicle() const
{
	return (RGLocation - (RGUpVector * AccelerationCenterOfMassOffset.Y)) +
		((RGForwardVector * (InfluencialDirection)) * AccelerationCenterOfMassOffset.X);
}


void AAVBaseVehicle::SetThrottleInput(float InputAxis)
{
	CurrentThrottleAxis = FMath::Clamp(InputAxis, 0.f, 1.f);
}


void AAVBaseVehicle::SetSteeringInput(float InputAxis)
{
	CurrentSteeringAxis = FMath::Clamp(InputAxis, -1.f,1.f);
}


void AAVBaseVehicle::SetBrakeInput(float InputAxis)
{
	CurrentBrakeAxis = FMath::Clamp(InputAxis, -1.f, 0.f);
}


void AAVBaseVehicle::ApplyInputStack(float DeltaTime)
{
	bIsGearReady = (GetWorld()->GetTimeSeconds() - SwapGearDirectionDelay) > LastTimeGearSwapped;

	ApplyThrottleInput(DeltaTime);
	ApplySteeringInput(DeltaTime);
	ApplyReverseInput(DeltaTime);
	ApplyBrakeInput(DeltaTime);

	// Compute influencial direction based on axis input
	InfluencialDirection = CurrentThrottleAxis + CurrentBrakeAxis;
}


void AAVBaseVehicle::ApplyThrottleInput(float DeltaTime)
{
	// Throttle
	if ((CurrentThrottleAxis >= 0.1f && bIsGearReady) || bIsBoosting)
	{
		AccelerationAccumulatedTime += DeltaTime;
		const float MaxThrottleRatio = bIsBoosting ? 1.f : CurrentThrottleAxis;
		if (LocalVelocity.X <= (GetComputedSpeed() * MaxThrottleRatio) && bIsMovingOnGround)
		{
			ThrottleForce = FVector::VectorPlaneProject(RGForwardVector, AvgedNormals) * getAcceleration();
		}
	}
}


void AAVBaseVehicle::ApplyReverseInput(float DeltaTime)
{
	// Reverse
	if (CurrentBrakeAxis <= -0.1 && !bIsBoosting && bIsGearReady)
	{
		AccelerationAccumulatedTime -= DeltaTime;
		if (LocalVelocity.X > (GetComputedSpeed() * FMath::Abs(CurrentBrakeAxis)) && bIsMovingOnGround)
		{
			ThrottleForce = FVector::VectorPlaneProject(RGForwardVector, AvgedNormals) * -getAcceleration();
		}
	}
}


void AAVBaseVehicle::ApplySteeringInput(float DeltaTime)
{
	if (FMath::Abs(CurrentSteeringAxis) >= 0.1f)
	{
 		const float DirectionFactor = (LastUsedGroundGear == ESimplifiedDirection::Reverse) ? -1.f : 1.f;
		if (bCompletelyInTheAir)
		{
			//#todo: Find a way to clamp torque speed while in air (ask designer what to do with the curve).
			const float ControlRatio = AirControlCurve ? AirControlCurve->GetFloatValue(TimeFalling) : 1.f;
			if (bOrientRotationToMovementInAir)
			{
				RigidHandle->AddForce(((FVector::DotProduct(RGRightVector, CurrentHorizontalVelocity) * -1.f) * (ORIENT_ROTATION_VELOCITY_MAX_RATE * OrientRotationToMovementInAirInfluenceRate)) * RGRightVector, false);
			}
			else
			{
				RigidHandle->AddForce(CurrentSteeringAxis * RGRightVector * AirStrafeAcceleration * ControlRatio, false);
			}
			SteeringForce = RGUpVector * DirectionFactor * CurrentSteeringAxis * AirTorqueAcceleration * ControlRatio;
		}
		else
		{
			// Removing error to compute the steering (therefore expected 0.f is 0.f and not 0.0001)
			const float ApproximateForwardVelocity = FMath::FloorToFloat(FMath::Abs(LocalVelocity.X));
			const float TargetSteerSpeed = SteeringActionCurve ? SteeringActionCurve->GetFloatValue(ApproximateForwardVelocity) : 1.f;
			if (FMath::Abs(CurrentAngularSpeed) < (TargetSteerSpeed * FMath::Abs(CurrentSteeringAxis)))
			{
				SteeringForce = RGUpVector * DirectionFactor * CurrentSteeringAxis * TorqueAcceleration;
			}
		}
	}
}


void AAVBaseVehicle::ApplyBrakeInput(float DeltaTime)
{
	// Compute Braking
	if (IsBraking() && bIsMovingOnGround && !bIsBoosting)
	{
		if (LocalVelocity.X <= 0.0)
		{
			ThrottleForce = (ThrottleForce * AccelerationInfluenceRateWhileBraking) + FVector::VectorPlaneProject(RGForwardVector, AvgedNormals) * CurrentThrottleAxis * BrakinDeceleration;
		}
		else
		{
			ThrottleForce = (ThrottleForce * AccelerationInfluenceRateWhileBraking) + FVector::VectorPlaneProject(RGForwardVector, AvgedNormals) * CurrentBrakeAxis * BrakinDeceleration;
		}
	}
}


float AAVBaseVehicle::GetAbsMaxSpeedAxisIndependent() const
{
	return FMath::Abs(LocalVelocity.X >= 0 ? getMaxSpeed() : getMaxBackwardsSpeed());
}


float AAVBaseVehicle::GetComputedSpeed() const
{
	if (!EngineAccelerationCurve)
	{
		return bIsBoosting ? MaxSpeed : MaxSpeed * AccelerationAccumulatedTime * CurrentGroundScalarResistance;
	}

	return bIsBoosting ? MaxSpeed : EngineAccelerationCurve->GetFloatValue(AccelerationAccumulatedTime) * CurrentGroundScalarResistance;
}


float AAVBaseVehicle::GetDecelerationRatio() const
{
	return EngineDecelerationCurve ? EngineDecelerationCurve->GetFloatValue(DecelerationAccumulatedTime) : DecelerationAccumulatedTime;
}


float AAVBaseVehicle::getMaxSpeed() const
{
	return MaxSpeed;
}


float AAVBaseVehicle::getMaxBackwardsSpeed() const
{
	return MaxBackwardsSpeed;
}


float AAVBaseVehicle::getAcceleration() const
{
	return VehicleAcceleration;
}


void AAVBaseVehicle::SetBoosting(bool inBoost)
{
	bIsBoosting = inBoost;
	MaxSpeed = bIsBoosting ? MaxSpeedBoosting : EngineAccelerationCurve ? EngineAccelerationCurve->FloatCurve.GetLastKey().Value : 2000.f;
	VehicleAcceleration = bIsBoosting ? VehicleBoostAcceleration : VehicleAcceleration;
}


void AAVBaseVehicle::ResetVehicle()
{
	LastTimeGearSwapped = -100.f; // We initialize this variable at a very low value to skip the gear ready delay when the kart starts moving without a defined gear
	WheelAnimData = FWheelAnimationData();
	TimeFalling = 0.f;
	SetBoosting(false);
	AccelerationAccumulatedTime = 0.f;
	if (RootBodyInstance)
	{
		DecelerationAccumulatedTime = MaxDecelerationCurveTime;
		RootBodyInstance->SetLinearVelocity(FVector::ZeroVector, false);
		RootBodyInstance->SetAngularVelocityInRadians(FVector::ZeroVector, false);
	}
}


bool AAVBaseVehicle::GetStickyWheels() const
{
	return bStickyWheels;
}


void AAVBaseVehicle::SetStickyWheels(bool inStickyWheels)
{
	bStickyWheels = inStickyWheels;
}


void AAVBaseVehicle::ApplyGravityForce(float DeltaTime)
{
	const bool bUseAvgedNormals = (bStickyWheels && bIsCloseToGround && !AvgedNormals.IsZero());
	const FVector VehicleAlignDirection = bUseAvgedNormals ? AvgedNormals : FVector::UpVector;

	// Gravity force
	FVector GravityForce = VehicleAlignDirection * (bIsMovingOnGround ? GravityGround : GravityAir);
	RigidHandle->AddForce(GravityForce, false);


	// Air control + unflipping forces + predictive landing
	if (bCompletelyInTheAir)
	{
		TimeFalling += DeltaTime;
		FVector TargetUpVector = VehicleAlignDirection;
		
		if (AirNavigationMode == EAirNavigationMode::Predictive)
		{
			const bool bPredictionSucceed = KartBallisticPrediction(RGLocation, CurrentHorizontalVelocity, ECollisionChannel::ECC_WorldDynamic, 7.f, 1.f, TargetUpVector);
			const float DotProductVehicle = FVector::DotProduct(RGUpVector, TargetUpVector);
			TargetUpVector = (bPredictionSucceed && DotProductVehicle > 0.5f) ? TargetUpVector : VehicleAlignDirection;
		}
		else if (AirNavigationMode == EAirNavigationMode::GroundAdaptative)
		{
			TargetUpVector = AvgedNormals;
			const float DotProductAlign = FVector::DotProduct(VehicleAlignDirection, TargetUpVector);
			TargetUpVector = (bIsCloseToGround && !AvgedNormals.IsZero() && DotProductAlign > 0.5f) ? TargetUpVector : VehicleAlignDirection;
		}
		
		const float FinalDotProductVehicle = FVector::DotProduct(RGUpVector, TargetUpVector);
		
		if(bFlipping) 
		{
			bFlipping = (FinalDotProductVehicle < 0.9f);
		}
		else
		{
			bFlipping = (FinalDotProductVehicle < -0.3f);
		}
		// If we are flipping we apply a string torque force, otherwise we let the user select.
		const float AntiRollMagnitude = bFlipping ? 250.f : AirNavigationAdaptativeForce;
		const FVector AntiRollForce = FVector::CrossProduct(TargetUpVector, -RGUpVector) * AntiRollMagnitude;
		SteeringForce += AntiRollForce;
	}
}


bool AAVBaseVehicle::KartBallisticPrediction(
	FVector StartPos,
	FVector LaunchVelocity,
	TEnumAsByte<ECollisionChannel> TraceChannel,
	float SimFrequency,
	float MaxSimTime,
	FVector& OutNormal)
{
	bool bBlockingHit = false;
	FHitResult FinalHit;
	FinalHit.Init();

	UWorld *MyWorld = GetWorld();

	if (SimFrequency > KINDA_SMALL_NUMBER && MyWorld)
	{
		const float SubstepDeltaTime = 1.f / SimFrequency;
		const FVector GravityForce = ((bStickyWheels && bIsCloseToGround && !AvgedNormals.IsZero()) ? AvgedNormals : FVector::UpVector) * GravityAir;
		
		FCollisionQueryParams QueryParams(SCENE_QUERY_STAT(KartBallisticPrediction));
		QueryParams.AddIgnoredActor(this);

		FVector CurrentVel = LaunchVelocity;
		FVector TraceStart = StartPos;
		FVector TraceEnd = TraceStart;
		float CurrentTime = 0.f;

		FHitResult ChannelTraceHit(NoInit);

		while (CurrentTime < MaxSimTime)
		{
			// Limit step to not go further than total time.
			const float ActualStepDeltaTime = FMath::Min(MaxSimTime - CurrentTime, SubstepDeltaTime);
			CurrentTime += ActualStepDeltaTime;

			#if ENABLE_DRAW_DEBUG
			if (bDebugInfo)
			{
				UKismetSystemLibrary::DrawDebugSphere(this, TraceStart, 10.f, 12, FLinearColor::Red, 0.f);
			}
			#endif
			
			// Integrate (Velocity Verlet method)
			TraceStart = TraceEnd;
			const FVector OldVelocity = CurrentVel;
			CurrentVel = OldVelocity + (GravityForce * ActualStepDeltaTime);
			TraceEnd = TraceStart + (OldVelocity + CurrentVel) * (0.5f * ActualStepDeltaTime);

			// fixmevori: Consider replacing by linetrace due to thin sphere?
			const bool bHit = MyWorld->SweepSingleByChannel(ChannelTraceHit, TraceStart, TraceEnd, FQuat::Identity, TraceChannel, FCollisionShape::MakeSphere(2.f), QueryParams);

			// See if there were any hits.
			if (bHit)
			{
				// Hit! We are done. Choose trace with earliest hit time.
				FinalHit = ChannelTraceHit;
				bBlockingHit = true;
				break;
			}
		}

		OutNormal = FinalHit.Normal;
	}

	return bBlockingHit;
}


FVector AAVBaseVehicle::CalcSuspensionSimulatedProxy(FVector RelativeOffset, const FSuspensionData& SuspensionData)
{
#if WITH_EDITOR
	ensureMsgf(!IsLocallyControlled(), TEXT("CalcSuspensionSimulatedProxy cannot be called from Locally Controlled Clients."));
#endif
	RGWorldTransform = RootBodyInstance->GetUnrealWorldTransform_AssumesLocked();
	RGUpVector = RGWorldTransform.GetUnitAxis(EAxis::Z);
	const float SuspensionLength = SuspensionData.SuspensionMaxDrop + SuspensionData.SuspensionMaxRaise;
	const FVector WheelRestingWorldLocation = RGWorldTransform.TransformPosition(RelativeOffset);
	const FVector TraceStart = WheelRestingWorldLocation + RGUpVector * SuspensionData.SuspensionMaxRaise;
	const FVector TraceEnd = WheelRestingWorldLocation - RGUpVector * (GroundDetectionDistanceThreshold);
	const float TraceFullLength = SuspensionLength + SuspensionData.WheelRadius;

	float Displacement = TraceFullLength - SuspensionData.WheelRadius;

	FHitResult OutHit;

	if (TraceFunc(TraceStart, TraceEnd, SuspensionData.TraceHalfSize, bDebugInfo > 0 ? EDrawDebugTrace::ForOneFrame : EDrawDebugTrace::None, OutHit))
	{
		Displacement = FMath::Min(OutHit.Distance, TraceFullLength) - SuspensionData.WheelRadius;
	}

	return RGUpVector * (SuspensionData.SuspensionMaxRaise - Displacement);
}


void AAVBaseVehicle::WheelsVisuals(FVector& FR, FRotator& FRR, FVector& FL, FRotator& FLR, FVector& RR, FRotator& RRR, FVector& RL, FRotator& RLR)
{
	if (CachedSuspensionInfo.Num() == NUMBER_OF_WHEELS)
	{
		// Wheel IK
		FL = RGUpVector * (CachedSuspensionInfo[FRONT_LEFT].SuspensionData.SuspensionMaxRaise - CachedSuspensionInfo[FRONT_LEFT].DisplacementInput);
		FR = RGUpVector * (CachedSuspensionInfo[FRONT_RIGHT].SuspensionData.SuspensionMaxRaise - CachedSuspensionInfo[FRONT_RIGHT].DisplacementInput);
		RL = RGUpVector * (CachedSuspensionInfo[BACK_LEFT].SuspensionData.SuspensionMaxRaise - CachedSuspensionInfo[BACK_LEFT].DisplacementInput);
		RR = RGUpVector * (CachedSuspensionInfo[BACK_RIGHT].SuspensionData.SuspensionMaxRaise - CachedSuspensionInfo[BACK_RIGHT].DisplacementInput);

		// Wheel Spin and Steer
		FLR = FRotator(FMath::RadiansToDegrees(WheelAnimData.WheelAngularPosition.X), WheelAnimData.WheelCurrentSteer.X, 0.f);
		FRR = FLR;
		RLR = FRotator(FMath::RadiansToDegrees(WheelAnimData.WheelAngularPosition.Y), WheelAnimData.WheelCurrentSteer.Y, 0.f);
		RRR = RLR;
	}
	else if (GetWorld()->IsGameWorld())
	{
		FL =  CalcSuspensionSimulatedProxy(FrontLeft, SuspensionFront);
		FR =  CalcSuspensionSimulatedProxy(FrontRight, SuspensionFront);
		RL =  CalcSuspensionSimulatedProxy(BackLeft, SuspensionRear);
		RR =  CalcSuspensionSimulatedProxy(BackRight, SuspensionRear);
	}
}


float AAVBaseVehicle::GetTerminalSpeed() const
{
	return TerminalSpeed;
}


bool AAVBaseVehicle::IsBraking() const
{
	// A vehicle is considered to be braking if the intended input direction goes against the current speed
	return (CurrentBrakeAxis <= -0.1f && LocalVelocity.X > StopThreshold) ||
		(CurrentThrottleAxis >= 0.1f && LocalVelocity.X < -StopThreshold);
}


ESimplifiedDirection AAVBaseVehicle::GetSimplifiedKartDirection() const
{
	return FMath::Abs(LocalVelocity.X) <= StopThreshold ? ESimplifiedDirection::Idle :
		(LocalVelocity.X > 0) ? ESimplifiedDirection::Forward : ESimplifiedDirection::Reverse;
}


FBodyInstance* AAVBaseVehicle::GetBodyInstance(UPrimitiveComponent* PrimitiveComponent)
{
	if (PrimitiveComponent == NULL) 
	{
		return NULL;
	}
	return PrimitiveComponent->GetBodyInstance();
}


#if WITH_EDITOR
void AAVBaseVehicle::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);

	const FProperty* PropertyThatChanged = PropertyChangedEvent.MemberProperty;
	if (PropertyThatChanged)
	{	
		if (PropertyThatChanged->GetFName() == GET_MEMBER_NAME_CHECKED(AAVBaseVehicle, bHideHelpHandlersInPIE))
		{
			BackRightHandle->SetHiddenInGame(bHideHelpHandlersInPIE);
			FrontRightHandle->SetHiddenInGame(bHideHelpHandlersInPIE);
			FrontLeftHandle->SetHiddenInGame(bHideHelpHandlersInPIE);
			BackLeftHandle->SetHiddenInGame(bHideHelpHandlersInPIE);
		}

		if (PropertyThatChanged->GetFName() == GET_MEMBER_NAME_CHECKED(FSuspensionData, TraceHalfSize))
		{
			// Since it's a half size we need to enforce full size when scaling the 1M box
			BackRightHandle->SetRelativeScale3D(FVector(SuspensionRear.TraceHalfSize.X * 2, SuspensionRear.TraceHalfSize.Y * 2, SuspensionRear.WheelRadius * 2) / 100.f);
			FrontRightHandle->SetRelativeScale3D(FVector(SuspensionFront.TraceHalfSize.X * 2, SuspensionFront.TraceHalfSize.Y * 2, SuspensionFront.WheelRadius * 2) / 100.f);
			FrontLeftHandle->SetRelativeScale3D(FVector(SuspensionFront.TraceHalfSize.X * 2, SuspensionFront.TraceHalfSize.Y * 2, SuspensionFront.WheelRadius * 2) / 100.f);
			BackLeftHandle->SetRelativeScale3D(FVector(SuspensionRear.TraceHalfSize.X * 2, SuspensionRear.TraceHalfSize.Y * 2, SuspensionRear.WheelRadius * 2) / 100.f);
		}
	}
}


void AAVBaseVehicle::PreSave(const class ITargetPlatform* TargetPlatform)
{
	Super::PreSave(TargetPlatform);
	BackRight = BackRightHandle->GetRelativeLocation();
	FrontRight = FrontRightHandle->GetRelativeLocation();
	FrontLeft = FrontLeftHandle->GetRelativeLocation();
	BackLeft = BackLeftHandle->GetRelativeLocation();
	
	// Ensure values in the CDO are correct
	UpdateHandlersTransformCDO();
}


void AAVBaseVehicle::OnConstruction(const FTransform& Transform)
{
	// We won't allow the user to scale the handlers
	BackRightHandle->SetRelativeScale3D(FVector(SuspensionRear.TraceHalfSize.X * 2, SuspensionRear.TraceHalfSize.Y * 2, SuspensionRear.WheelRadius * 2) / 100.f);
	FrontRightHandle->SetRelativeScale3D(FVector(SuspensionFront.TraceHalfSize.X * 2, SuspensionFront.TraceHalfSize.Y * 2, SuspensionFront.WheelRadius * 2) / 100.f);
	FrontLeftHandle->SetRelativeScale3D(FVector(SuspensionFront.TraceHalfSize.X * 2, SuspensionFront.TraceHalfSize.Y * 2, SuspensionFront.WheelRadius * 2) / 100.f);
	BackLeftHandle->SetRelativeScale3D(FVector(SuspensionRear.TraceHalfSize.X * 2, SuspensionRear.TraceHalfSize.Y * 2, SuspensionRear.WheelRadius * 2) / 100.f);

	// We don't allow the user to rotate the handlers
	BackRightHandle->SetRelativeRotation(FRotator::ZeroRotator);
	FrontRightHandle->SetRelativeRotation(FRotator::ZeroRotator);
	FrontLeftHandle->SetRelativeRotation(FRotator::ZeroRotator);
	BackLeftHandle->SetRelativeRotation(FRotator::ZeroRotator);
}


void AAVBaseVehicle::UpdateHandlersTransformCDO()
{
	BackRightHandle->SetRelativeTransform(FTransform(FRotator::ZeroRotator, BackRight, FVector(SuspensionRear.TraceHalfSize.X * 2, SuspensionRear.TraceHalfSize.Y * 2, SuspensionRear.WheelRadius * 2) / 100.f));
	FrontRightHandle->SetRelativeTransform(FTransform(FRotator::ZeroRotator, FrontRight, FVector(SuspensionFront.TraceHalfSize.X * 2, SuspensionFront.TraceHalfSize.Y * 2, SuspensionFront.WheelRadius * 2) / 100.f));
	FrontLeftHandle->SetRelativeTransform(FTransform(FRotator::ZeroRotator, FrontLeft, FVector(SuspensionFront.TraceHalfSize.X * 2, SuspensionFront.TraceHalfSize.Y * 2, SuspensionFront.WheelRadius * 2) / 100.f));
	BackLeftHandle->SetRelativeTransform(FTransform(FRotator::ZeroRotator, BackLeft, FVector(SuspensionRear.TraceHalfSize.X * 2, SuspensionRear.TraceHalfSize.Y * 2, SuspensionRear.WheelRadius * 2) / 100.f));
}
#endif // WITH_EDITOR


// Called whenever this actor is being removed from a level
void AAVBaseVehicle::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	UWorld* World = GetWorld();
	if (World)
	{
		FPhysScene* PScene = World->GetPhysicsScene();
		if (PScene)
		{
			// Unregister physics step delegate
			PScene->UnregisterAsyncPhysicsTickActor(this);
		}
	}

	Super::EndPlay(EndPlayReason);
}

void AAVBaseVehicle::AsyncPhysicsTickActor(float DeltaTime, float SimTime)
{
	Super::AsyncPhysicsTickActor(DeltaTime, SimTime);
	PhysicsTick(DeltaTime);
}


// Called to bind functionality to input
void AAVBaseVehicle::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);
	PlayerInputComponent->BindAxis("MoveForward", this, &AAVBaseVehicle::SetThrottleInput);
	PlayerInputComponent->BindAxis("MoveRight", this, &AAVBaseVehicle::SetSteeringInput);
	PlayerInputComponent->BindAxis("Brake", this, &AAVBaseVehicle::SetBrakeInput);
}


void AAVBaseVehicle::InitVehicle()
{
	CachedSuspensionInfo.Init(FCachedSuspensionInfo(), NUMBER_OF_WHEELS);
	CachedSuspensionInfo[FRONT_LEFT].SuspensionData = SuspensionFront;
	CachedSuspensionInfo[FRONT_LEFT].WheelRelativeLocation = FrontLeft;
	CachedSuspensionInfo[FRONT_RIGHT].SuspensionData = SuspensionFront;
	CachedSuspensionInfo[FRONT_RIGHT].WheelRelativeLocation = FrontRight;
	CachedSuspensionInfo[BACK_LEFT].SuspensionData = SuspensionRear;
	CachedSuspensionInfo[BACK_LEFT].WheelRelativeLocation = BackLeft;
	CachedSuspensionInfo[BACK_RIGHT].SuspensionData = SuspensionRear;
	CachedSuspensionInfo[BACK_RIGHT].WheelRelativeLocation = BackRight;
	
	SetupVehicleCurves();

	TArray<FVector> LocalSpringPositions = { FrontLeft, FrontRight, BackLeft, BackRight }; // Order relevant
	TArray<float> OutSprungMasses;
	FSuspensionUtility::ComputeSprungMasses(LocalSpringPositions, RootBodyInstance->GetMassOverride(), OutSprungMasses);

	// Calculate spring damping values we will use for physics simulation from the normalized damping ratio
	for (int SpringIdx = 0; SpringIdx < NUMBER_OF_WHEELS; SpringIdx++)
	{
		auto& Susp = CachedSuspensionInfo[SpringIdx];

		float NaturalFrequency = FSuspensionUtility::ComputeNaturalFrequency(Susp.SuspensionData.SpringRate, OutSprungMasses[SpringIdx]);
		float Damping = FSuspensionUtility::ComputeDamping(Susp.SuspensionData.SpringRate, OutSprungMasses[SpringIdx], Susp.SuspensionData.DampingRatio);

		Susp.SuspensionLength = Susp.SuspensionData.SuspensionMaxDrop + Susp.SuspensionData.SuspensionMaxRaise;
		Susp.ReboundDamping = Damping;
		Susp.BoundDamping = Damping;
		Susp.RestingForce = OutSprungMasses[SpringIdx] * -GravityGround;
	}

	// DEPRECATION AREA --
#if WITH_EDITOR
#endif

}


void AAVBaseVehicle::SetupVehicleCurves()
{
	// Initializing acceleration curves
	if (EngineAccelerationCurve)
	{
		MaxAccelerationCurveTime = EngineAccelerationCurve->FloatCurve.GetLastKey().Time;
		MinAccelerationCurveTime = EngineAccelerationCurve->FloatCurve.GetFirstKey().Time;
		MaxSpeed = EngineAccelerationCurve->FloatCurve.GetLastKey().Value;
		MaxBackwardsSpeed = EngineAccelerationCurve->FloatCurve.GetFirstKey().Value;
	}
	else
	{
		MaxAccelerationCurveTime = 1.f;
		MinAccelerationCurveTime = -1.f;
		MaxSpeed = 2000.f;
		MaxBackwardsSpeed = -2000.f;
		UE_LOG(LogTemp, Warning, TEXT("EngineAccelerationCurve missing, assuming default values."));
	}

	if (EngineDecelerationCurve)
	{
		MaxDecelerationCurveTime = EngineDecelerationCurve->FloatCurve.GetLastKey().Time;
	}
	else
	{
		MaxDecelerationCurveTime = 1.f;
		UE_LOG(LogTemp, Warning, TEXT("EngineDecelerationCurve missing, assuming default values."));
	}

	if (!AirControlCurve)
	{
		UE_LOG(LogTemp, Warning, TEXT("AirControlCurve missing, assuming default values."));
	}


#if WITH_EDITOR
	ensureMsgf(FMath::Max(FMath::Max(MaxSpeed, FMath::Abs(MaxBackwardsSpeed)), MaxSpeedBoosting) < TerminalSpeed, TEXT("Terminal Speed is lower than the max speed values."));
#endif
}
