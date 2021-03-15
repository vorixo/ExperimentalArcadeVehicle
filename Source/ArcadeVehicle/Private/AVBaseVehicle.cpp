// Fill out your copyright notice in the Description page of Project Settings.


#include "AVBaseVehicle.h"
#include "CollisionQueryParams.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "ChaosVehicles/ChaosVehiclesCore/Public/SuspensionUtility.h"

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
	GravityAir = -980.f;
	GravityGround = -980.f;
	
	// Gameplay driven friction
	LateralFrictionModifier = 1.f;
	ScalarFrictionVal = 1.f; // DEPRECATED

	bStickyWheels = false;
	// Default to z grav
	AvgedNormals = FVector::UpVector;
	LinearDampingAir = 0.01f;
	AngularDampingGround = 10.0f;
	MaxSpeedBoosting = 3000.f;
	CurrentAngularSpeed = 0.f;
	CurrentHorizontalSpeed = 0.f;
	// Maximum speed that can ever be achieved, accelerating or not.
	TerminalSpeed = 3500.f;
	CurrentThrottleAxis = 0.f;
	CurrentBrakeAxis = 0.f;
	bTiltedThrottle = true;
	VehicleAcceleration = 5000.f;
	VehicleBoostAcceleration = 3000.f;
	TorqueSpeed = 12.f;
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
	AirYawSpeed = 20.f;
	AirStrafeSpeed = 1000.f;
	bOrientRotationToMovementInAir = false;
	OrientRotationToMovementInAirInfluenceRate = 1.f;
	AccelerationInfluenceRateWhileBraking = 0.3f;
	SteeringDecelerationSideVelocityFactorScale = 40.f;
	SteeringDecelerationSideVelocityInterpolationSpeed = 0.4;
	SwapGearDirectionDelay = 0.4f;
	StopThreshold = 10.f;
	InfluencialDirection = 0.f;
	bIsGearReady = true;
	AirNavigationMode = EAirNavigationMode::None;
	bFlipping = false;
	ResetVehicle();

	CollisionMesh = CreateDefaultSubobject<UStaticMeshComponent>("CollisionMesh");
	if (CollisionMesh)
	{
		CollisionMesh->SetSimulatePhysics(true);
		CollisionMesh->BodyInstance.bOverrideMass = true;
		CollisionMesh->BodyInstance.SetMassOverride(1.3f);
		CollisionMesh->SetEnableGravity(false);
		CollisionMesh->bReplicatePhysicsToAutonomousProxy = false;
		CollisionMesh->BodyInstance.COMNudge = FVector(0.f, 0.f, -20.f);
		CollisionMesh->SetCollisionProfileName(UCollisionProfile::PhysicsActor_ProfileName);
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
		static ConstructorHelpers::FObjectFinder<UStaticMesh> CylinderHelpMesh(TEXT("StaticMesh'/ArcadeVehicle/HelperCube.HelperCube'"));
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
	PawnRootComponent = Cast<UPrimitiveComponent>(GetRootComponent());
	
	if (PawnRootComponent != NULL)
	{
		RootBodyInstance = PawnRootComponent->GetBodyInstance();

		UWorld* World = GetWorld();
		if (World->IsGameWorld() && IsLocallyControlled())
		{
			FPhysScene* PScene = World->GetPhysicsScene();
			if (PScene)
			{
				// Register physics step delegate
				OnPhysSceneStepHandle = PScene->OnPhysSceneStep.AddUObject(this, &AAVBaseVehicle::PhysSceneStep);
				InitVehicle();
			}
		}
	}

	Super::BeginPlay();
}


// Called by OnCalculateCustomPhysics delegate when physics update is initiated
void AAVBaseVehicle::PhysSceneStep(FPhysScene* PhysScene, float DeltaTime)
{
	PhysicsTick(DeltaTime);
}


void AAVBaseVehicle::PhysicsTick(float SubstepDeltaTime)
{
	if (!RootBodyInstance) return;

	// DEPRECATION AREA:
	LateralFrictionModifier = ScalarFrictionVal; // DEPRECATED

	// Getting the transformation matrix of the object
	RGWorldTransform = RootBodyInstance->GetUnrealWorldTransform_AssumesLocked();

	// World Location
	RGLocation = RGWorldTransform.GetLocation();

	// Getting the forward, right, and up vectors
	RGForwardVector = RGWorldTransform.GetUnitAxis(EAxis::X);
	RGRightVector = RGWorldTransform.GetUnitAxis(EAxis::Y);
	RGUpVector = RGWorldTransform.GetUnitAxis(EAxis::Z);


	// Calc velocities and Speed
	CurrentHorizontalVelocity = FVector::VectorPlaneProject(RootBodyInstance->GetUnrealWorldVelocity(), RGUpVector);
	CurrentHorizontalSpeed = FMath::Sign(FVector::DotProduct(CurrentHorizontalVelocity, RGForwardVector)) * CurrentHorizontalVelocity.Size();
	CurrentAngularVelocity = RootBodyInstance->GetUnrealWorldAngularVelocityInRadians();
	CurrentAngularSpeed = FMath::Sign(FVector::DotProduct(CurrentAngularVelocity, RGUpVector)) * CurrentAngularVelocity.Size();
	LocalVelocity = RGWorldTransform.InverseTransformVectorNoScale(CurrentHorizontalVelocity);

	// Moving on top of moving surfaces
	ComputeBasedMovement();

	// Input processing
	ApplyInputStack(SubstepDeltaTime);


	ApplySuspensionForces(SubstepDeltaTime);

	/************************************************************************/
	/* Apply gravity and sliding forces                                     */
	/************************************************************************/
	ApplyGravityForce(SubstepDeltaTime);

	if (bIsMovingOnGround)
	{
		// Drag/Friction force
		RootBodyInstance->AddForce(GetLateralFrictionDragForce(), false, false);
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
		const FVector side_velocity = FMath::Abs(LocalVelocity.Y) * RGForwardVector;
		const FVector non_forward_velocity = CurrentHorizontalVelocity.GetSafeNormal2D() * getMaxSpeed() - (side_velocity * SteeringDecelerationSideVelocityFactorScale);
		const bool bTurnDeceleration = CurrentHorizontalSpeed > non_forward_velocity.Size() && !bIsBoosting && !FMath::IsNearlyZero(CurrentSteeringAxis);

		if (bTurnDeceleration)
		{
			ThrottleForce = (non_forward_velocity.GetSafeNormal2D() * -CurrentHorizontalSpeed * SteeringDecelerationSideVelocityInterpolationSpeed);
		}

		const float SpeedRatio = CurrentHorizontalSpeed / GetAbsMaxSpeedAxisIndependent();
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
		const bool bIsGoingOverLegalSpeed = (FMath::Abs(CurrentHorizontalSpeed) > ((GetAbsMaxSpeedAxisIndependent() * CurrentGroundScalarResistance) + LegalSpeedOffset));
		ThrottleForce = bIsGoingOverLegalSpeed ? FVector::VectorPlaneProject(-CurrentHorizontalVelocity.GetSafeNormal(), AvgedNormals) * getAcceleration() : ThrottleForce;

		// Adjusting Throttle based on engine decceleration values
		ThrottleForce = bIsMovingOnGround ? ThrottleForce - ThrottleAdjustmentRatio : ThrottleForce;
	}

		
	// Forward and backwards speed work with 0 damping.
	RootBodyInstance->LinearDamping = bIsMovingOnGround ? 0.f : LinearDampingAir;
	RootBodyInstance->AngularDamping = bCompletelyInTheAir ? 0.f : AngularDampingGround;
	RootBodyInstance->UpdateDampingProperties();


	/************************************************************************/
	/* Apply movement forces                                                */
	/************************************************************************/
	RootBodyInstance->AddTorqueInRadians(SteeringForce, false, true);
	
	if (bTiltedThrottle)
	{
		RootBodyInstance->AddForceAtPosition(ThrottleForce, GetOffsetedCenterOfVehicle(), false, false);
	}
	else
	{
		RootBodyInstance->AddForce(ThrottleForce, false);
	}
	
	// Vehicles can't go beyond the terminal speed. This code snippet also smooths out current velocity towards max speed leaving some room for physical enviro impulses
	if (FMath::Abs(CurrentHorizontalSpeed) > FMath::Min(GetAbsMaxSpeedAxisIndependent(), GetTerminalSpeed()))
	{
		const float BlendAlpha = (FMath::Abs(CurrentHorizontalSpeed) - GetAbsMaxSpeedAxisIndependent()) / (GetTerminalSpeed() - GetAbsMaxSpeedAxisIndependent());
		const float TerminalVelocityPreemptionFinalForce = FMath::Abs(CurrentHorizontalSpeed) > GetTerminalSpeed() ? BIG_NUMBER : FMath::Lerp(0.f, GetTerminalSpeed() + TERMINAL_VELOCITY_PREEMPTION_FORCE_OFFSET, BlendAlpha);
		RootBodyInstance->AddForce(-CurrentHorizontalVelocity.GetSafeNormal() * TerminalVelocityPreemptionFinalForce, false);
	}
	
	// We also want to help the vehicle get into an idle state
	if (FMath::Abs(CurrentHorizontalSpeed) <= StopThreshold)
	{
		const float BlendAlpha = FMath::Abs(CurrentHorizontalSpeed)/(StopThreshold);
		const float TerminalVelocityPreemptionFinalForce = FMath::Lerp(0.f, IDLE_VEHICLE_FORCE, BlendAlpha);
		RootBodyInstance->AddForce(-CurrentHorizontalVelocity.GetSafeNormal() * TerminalVelocityPreemptionFinalForce, false);
	}
	
	/************************************************************************/
	/* End apply movement forces                                            */
	/************************************************************************/

	if (bDebugInfo > 0)
	{
		UKismetSystemLibrary::DrawDebugSphere(this, GetOffsetedCenterOfVehicle(), 10.f, 12);
		UKismetSystemLibrary::DrawDebugSphere(this, RGLocation, 10.f, 12);
		UKismetSystemLibrary::DrawDebugString(this, RGLocation, FString::SanitizeFloat(CurrentHorizontalSpeed).Mid(0, 5), NULL,FLinearColor::Red);
	}

	// TODO: Vehicle Effects

	// Rest variables - Last tick calculations
	LastUsedGear = CurrentSimplifiedDirection;
	ThrottleForce = FVector::ZeroVector;
	SteeringForce = FVector::ZeroVector;
}


FSuspensionHitInfo AAVBaseVehicle::CalcSuspension(FVector RelativeOffset, FCachedSuspensionInfo& InCachedInfo, float DeltaTime)
{
	const FVector TraceStart = RGWorldTransform.TransformPositionNoScale(RelativeOffset);
	const FVector TraceEnd = TraceStart - (RGUpVector * (GroundDetectionDistanceThreshold));
	FSuspensionHitInfo sHitInfo;

	FHitResult OutHit;
	
	if (TraceFunc(TraceStart, TraceEnd, InCachedInfo.SuspensionData.TraceHalfSize, bDebugInfo > 0 ? EDrawDebugTrace::ForOneFrame : EDrawDebugTrace::None, OutHit))
	{
		InCachedInfo.DisplacementInput = FMath::Min(OutHit.Distance, InCachedInfo.SuspensionData.SuspensionLength);

		#if ENABLE_DRAW_DEBUG
		if (bDebugInfo)
		{
			UWorld* World = GEngine->GetWorldFromContextObject(this, EGetWorldErrorMode::LogAndReturnNull);
			if (World) 
			{
				UKismetSystemLibrary::DrawDebugString(World, TraceStart, FString::SanitizeFloat(InCachedInfo.DisplacementInput).Mid(0, 4), nullptr, FLinearColor::White, 0.f);
				DrawDebugSweptBox(GetWorld(), TraceStart, TraceStart - (RGUpVector * (InCachedInfo.SuspensionData.SuspensionLength)), RGWorldTransform.GetRotation(), 
					FVector(InCachedInfo.SuspensionData.TraceHalfSize.X, InCachedInfo.SuspensionData.TraceHalfSize.Y, 0.f), EDrawDebugTrace::ForOneFrame, InCachedInfo.DisplacementInput > 0.f ? FColor::Blue : FColor::Red, false, 0.f);
			}
		}
		#endif

		if (OutHit.Distance <= InCachedInfo.SuspensionData.SuspensionLength)
		{
			const float ForceMagnitude = GetSuspensionForceMagnitude(InCachedInfo, DeltaTime);
	
			InCachedInfo.LastDisplacement = InCachedInfo.DisplacementInput;

			// FIXMEVORI: We can also use here AvgedNormals for a suspension regulated result
			const FVector FinalForce = (OutHit.ImpactNormal * ForceMagnitude);
			
			RootBodyInstance->AddForceAtPosition(FinalForce, TraceStart, false);

			// Trace hits and it is within the suspension length size, hence the wheel is on the ground
			sHitInfo.bWheelOnGround = true;

			// Physical material properties for this wheel contacting the ground
			const bool bPhysMatExists = OutHit.PhysMaterial.IsValid();
			sHitInfo.GroundFriction = bPhysMatExists ? OutHit.PhysMaterial->Friction : DEFAULT_GROUND_FRICTION;
			sHitInfo.GroundResistance = bPhysMatExists ? OutHit.PhysMaterial->Density : DEFAULT_GROUND_RESISTANCE;
		}		
		sHitInfo.bTraceHit = true;

		InCachedInfo.ImpactNormal = OutHit.ImpactNormal;
		return sHitInfo;
	}

	return sHitInfo;
}


float AAVBaseVehicle::GetSuspensionForceMagnitude(const FCachedSuspensionInfo& InCachedInfo, float DeltaTime) const
{
	const float LocalWheelVelocity = (InCachedInfo.DisplacementInput - InCachedInfo.LastDisplacement)/DeltaTime;

	const float SpringDisplacement = InCachedInfo.SuspensionData.SuspensionLength - InCachedInfo.DisplacementInput;

	const float Damping = (InCachedInfo.DisplacementInput < InCachedInfo.LastDisplacement) ? InCachedInfo.BoundDamping : InCachedInfo.ReboundDamping;
	const float StiffnessForce = SpringDisplacement * InCachedInfo.SuspensionData.SpringRate;
	const float DampingForce = LocalWheelVelocity * Damping;
	const float SuspensionForce = StiffnessForce - DampingForce;

	const float ForceMagnitude = InCachedInfo.SuspensionData.SuspensionLoadRatio * SuspensionForce + (1.f - InCachedInfo.SuspensionData.SuspensionLoadRatio) * InCachedInfo.RestingForce;

	return ForceMagnitude;
}


FVector AAVBaseVehicle::GetLateralFrictionDragForce() const
{
	const FVector FrictionDragForce = ((FVector::DotProduct(RGRightVector, CurrentHorizontalVelocity) * -1.f) * (LateralFrictionModifier * CurrentGroundFriction * BASE_GROUND_FRICTION)) * RGRightVector;
	return FrictionDragForce;
}


void AAVBaseVehicle::ApplySuspensionForces(float DeltaTime)
{
	
	const FSuspensionHitInfo BackRightSuspension = CalcSuspension(BackRight, CachedSuspensionInfo[BACK_RIGHT], DeltaTime);
	const FSuspensionHitInfo FrontRightSuspension = CalcSuspension(FrontRight, CachedSuspensionInfo[FRONT_RIGHT], DeltaTime);
	const FSuspensionHitInfo FrontLeftSuspension = CalcSuspension(FrontLeft, CachedSuspensionInfo[FRONT_LEFT], DeltaTime);
	const FSuspensionHitInfo BackLeftSuspension = CalcSuspension(BackLeft, CachedSuspensionInfo[BACK_LEFT], DeltaTime);
	
	// On landing impl. If in the previous step it was in the air and now is in the ground 
	const bool bWasMovingOnGround = bIsMovingOnGround;

	const uint8 WheelsTouchingTheGround = (BackRightSuspension.bWheelOnGround + FrontRightSuspension.bWheelOnGround + FrontLeftSuspension.bWheelOnGround + BackLeftSuspension.bWheelOnGround);
	bIsMovingOnGround = WheelsTouchingTheGround > 2;
	bCompletelyInTheAir = WheelsTouchingTheGround == 0;
	bCompletelyInTheGround = WheelsTouchingTheGround == NUMBER_OF_WHEELS;
	bIsCloseToGround = (BackRightSuspension.bTraceHit + FrontRightSuspension.bTraceHit + FrontLeftSuspension.bTraceHit + BackLeftSuspension.bTraceHit) > 2;
	CurrentGroundFriction = (BackRightSuspension.GroundFriction + FrontRightSuspension.GroundFriction + FrontLeftSuspension.GroundFriction + BackLeftSuspension.GroundFriction) / NUMBER_OF_WHEELS;
	CurrentGroundScalarResistance = (BackRightSuspension.GroundResistance + FrontRightSuspension.GroundResistance + FrontLeftSuspension.GroundResistance + BackLeftSuspension.GroundResistance) / NUMBER_OF_WHEELS;
		
	AvgedNormals = FVector::ZeroVector;
	for (FCachedSuspensionInfo csi : CachedSuspensionInfo) AvgedNormals += csi.ImpactNormal;
	AvgedNormals /= NUMBER_OF_WHEELS;

	// On landed event
	if (!bWasMovingOnGround && bIsMovingOnGround)
	{
		Landed(AvgedNormals);
	}
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

			RootBodyInstance->SetBodyTransform(NewTransform, ETeleportType::None);
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
		if (CurrentHorizontalSpeed <= GetComputedSpeed() && bIsMovingOnGround)
		{
			const float MaxThrottleRatio = bIsBoosting ? 1.f : CurrentThrottleAxis;
			ThrottleForce = FVector::VectorPlaneProject(RGForwardVector, AvgedNormals) * getAcceleration() * MaxThrottleRatio;
		}
	}
}


void AAVBaseVehicle::ApplyReverseInput(float DeltaTime)
{
	// Reverse
	if (CurrentBrakeAxis <= -0.1 && !bIsBoosting && bIsGearReady)
	{
		AccelerationAccumulatedTime -= DeltaTime;
		if (CurrentHorizontalSpeed > GetComputedSpeed() && bIsMovingOnGround)
		{
			ThrottleForce = FVector::VectorPlaneProject(RGForwardVector, AvgedNormals) * CurrentBrakeAxis * getAcceleration();
		}
	}
}


void AAVBaseVehicle::ApplySteeringInput(float DeltaTime)
{
	// Steering
	if ((CurrentSteeringAxis >= 0.1f && CurrentAngularSpeed <= TorqueSpeed) ||
		(CurrentSteeringAxis <= -0.1f && CurrentAngularSpeed >= -TorqueSpeed))
	{
		// fixmevori: simplify
		// Direction Calc
		const float DirectionSign = FMath::Sign(CurrentHorizontalSpeed);
		const float DirectionFactor = (DirectionSign == 0 || (CurrentHorizontalSpeed <= 0 && CurrentHorizontalSpeed > -100 && !bIsMovingOnGround)) ? 1.f : DirectionSign;
		// Steering acceleration
		const float AlphaInputTorque = SteeringActionCurve ? SteeringActionCurve->GetFloatValue(FMath::Clamp(FMath::Abs(CurrentHorizontalSpeed) / 1000.f, 0.f, 1.f)) : 1.f;
		float InputTorqueRatio = FMath::Lerp(AlphaInputTorque, TorqueSpeed * CurrentSteeringAxis, AlphaInputTorque);

		if (bCompletelyInTheAir)
		{
			// Air steering input stack - fixmevori: experimental curve
			const float ControlRatio = AirControlCurve ? AirControlCurve->GetFloatValue(TimeFalling) : 1.f;
			InputTorqueRatio = CurrentSteeringAxis * AirYawSpeed * ControlRatio;

			if (bOrientRotationToMovementInAir)
			{

				ThrottleForce = ((FVector::DotProduct(RGRightVector, CurrentHorizontalVelocity) * -1.f) * (ORIENT_ROTATION_VELOCITY_MAX_RATE * OrientRotationToMovementInAirInfluenceRate)) * RGRightVector;
			}
			else
			{
				ThrottleForce = CurrentSteeringAxis * RGRightVector * AirStrafeSpeed * ControlRatio;
			}
		}
		SteeringForce = RGUpVector * DirectionFactor * InputTorqueRatio;
	}
}


void AAVBaseVehicle::ApplyBrakeInput(float DeltaTime)
{
	// Compute Braking
	if (IsBraking() && bIsMovingOnGround && !bIsBoosting)
	{
		if (CurrentHorizontalSpeed <= 0.0)
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
	return FMath::Abs(CurrentHorizontalSpeed >= 0 ? getMaxSpeed() : getMaxBackwardsSpeed());
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
	LastTimeGearSwapped = -100.f; // We initialise this variable at a very low value to skip the gear ready delay when the kart starts moving without a defined gear
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
	RootBodyInstance->AddForce(GravityForce, false, false);


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

		const float AntiRollMagnitude = bFlipping ? 2500.f : 200.f;
		const FVector AntiRollForce = FVector::CrossProduct(TargetUpVector, -RGUpVector) * AntiRollMagnitude;
		FVector AngularFinalForce = SteeringForce + AntiRollForce;
		RootBodyInstance->SetAngularVelocityInRadians(AngularFinalForce * 0.01, false);
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


float AAVBaseVehicle::GetTerminalSpeed() const
{
	return TerminalSpeed;
}


bool AAVBaseVehicle::IsBraking() const
{
	return (CurrentBrakeAxis <= -0.1 && CurrentHorizontalSpeed > StopThreshold) ||
		(CurrentThrottleAxis >= 0.1 && CurrentHorizontalSpeed < -StopThreshold);
}


ESimplifiedDirection AAVBaseVehicle::GetSimplifiedKartDirection() const
{
	return FMath::Abs(CurrentHorizontalSpeed) <= StopThreshold ? ESimplifiedDirection::Idle : 
		(CurrentHorizontalSpeed > 0) ? ESimplifiedDirection::Forward : ESimplifiedDirection::Reverse;
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
		
		if (PropertyThatChanged->GetFName() == GET_MEMBER_NAME_CHECKED(AAVBaseVehicle, SuspensionFront.SuspensionLength)
			|| PropertyThatChanged->GetFName() == GET_MEMBER_NAME_CHECKED(AAVBaseVehicle, SuspensionRear.SuspensionLength)
			|| PropertyThatChanged->GetFName() == GET_MEMBER_NAME_CHECKED(AAVBaseVehicle, SuspensionFront.TraceHalfSize)
			|| PropertyThatChanged->GetFName() == GET_MEMBER_NAME_CHECKED(AAVBaseVehicle, SuspensionRear.TraceHalfSize))
		{
			// Since it's a half size we need to enforce full size when scaling the 1M box
			BackRightHandle->SetRelativeScale3D( FVector(SuspensionRear.TraceHalfSize.X * 2, SuspensionRear.TraceHalfSize.Y * 2, SuspensionRear.SuspensionLength) / 100.f);
			FrontRightHandle->SetRelativeScale3D( FVector(SuspensionFront.TraceHalfSize.X * 2, SuspensionFront.TraceHalfSize.Y * 2, SuspensionFront.SuspensionLength) / 100.f);
			FrontLeftHandle->SetRelativeScale3D( FVector(SuspensionFront.TraceHalfSize.X * 2, SuspensionFront.TraceHalfSize.Y * 2, SuspensionFront.SuspensionLength) / 100.f);
			BackLeftHandle->SetRelativeScale3D( FVector(SuspensionRear.TraceHalfSize.X * 2, SuspensionRear.TraceHalfSize.Y * 2, SuspensionRear.SuspensionLength) / 100.f);
		}
		
		if (PropertyThatChanged->GetFName() == GET_MEMBER_NAME_CHECKED(AAVBaseVehicle, bHideHelpHandlersInPIE))
		{
			BackRightHandle->SetHiddenInGame(bHideHelpHandlersInPIE);
			FrontRightHandle->SetHiddenInGame(bHideHelpHandlersInPIE);
			FrontLeftHandle->SetHiddenInGame(bHideHelpHandlersInPIE);
			BackLeftHandle->SetHiddenInGame(bHideHelpHandlersInPIE);
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
	BackRightHandle->SetRelativeScale3D(FVector(SuspensionRear.TraceHalfSize.X * 2, SuspensionRear.TraceHalfSize.Y * 2, SuspensionRear.SuspensionLength) / 100.f);
	FrontRightHandle->SetRelativeScale3D(FVector(SuspensionFront.TraceHalfSize.X * 2, SuspensionFront.TraceHalfSize.Y * 2, SuspensionFront.SuspensionLength) / 100.f);
	FrontLeftHandle->SetRelativeScale3D(FVector(SuspensionFront.TraceHalfSize.X * 2, SuspensionFront.TraceHalfSize.Y * 2, SuspensionFront.SuspensionLength) / 100.f);
	BackLeftHandle->SetRelativeScale3D(FVector(SuspensionRear.TraceHalfSize.X * 2, SuspensionRear.TraceHalfSize.Y * 2, SuspensionRear.SuspensionLength) / 100.f);

	// We don't allow the user to rotate the handlers
	BackRightHandle->SetRelativeRotation(FRotator::ZeroRotator);
	FrontRightHandle->SetRelativeRotation(FRotator::ZeroRotator);
	FrontLeftHandle->SetRelativeRotation(FRotator::ZeroRotator);
	BackLeftHandle->SetRelativeRotation(FRotator::ZeroRotator);
}


void AAVBaseVehicle::UpdateHandlersTransformCDO()
{
	BackRightHandle->SetRelativeTransform(FTransform(FRotator::ZeroRotator, BackRight, FVector(SuspensionRear.TraceHalfSize.X * 2, SuspensionRear.TraceHalfSize.Y * 2, SuspensionRear.SuspensionLength) / 100.f));
	FrontRightHandle->SetRelativeTransform(FTransform(FRotator::ZeroRotator, FrontRight, FVector(SuspensionFront.TraceHalfSize.X * 2, SuspensionFront.TraceHalfSize.Y * 2, SuspensionFront.SuspensionLength) / 100.f));
	FrontLeftHandle->SetRelativeTransform(FTransform(FRotator::ZeroRotator, FrontLeft, FVector(SuspensionFront.TraceHalfSize.X * 2, SuspensionFront.TraceHalfSize.Y * 2, SuspensionFront.SuspensionLength) / 100.f));
	BackLeftHandle->SetRelativeTransform(FTransform(FRotator::ZeroRotator, BackLeft, FVector(SuspensionRear.TraceHalfSize.X * 2, SuspensionRear.TraceHalfSize.Y * 2, SuspensionRear.SuspensionLength) / 100.f));
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
			PScene->OnPhysSceneStep.Remove(OnPhysSceneStepHandle);
		}
	}

	Super::EndPlay(EndPlayReason);
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
	CachedSuspensionInfo[FRONT_RIGHT].SuspensionData = SuspensionFront;
	CachedSuspensionInfo[BACK_LEFT].SuspensionData = SuspensionRear;
	CachedSuspensionInfo[BACK_RIGHT].SuspensionData = SuspensionRear;
	
	SetupVehicleCurves();
	
	TArray<FVector> LocalSpringPositions = { FrontLeft - FVector(0,0,20.f) , BackLeft - FVector(0,0,20.f), FrontRight - FVector(0,0,20.f), BackRight - FVector(0,0,20.f) };
	TArray<float> OutSprungMasses;
	FSuspensionUtility::ComputeSprungMasses(LocalSpringPositions, RootBodyInstance->GetMassOverride(), OutSprungMasses);

	// Calculate spring damping values we will use for physics simulation from the normalized damping ratio
	for (int SpringIdx = 0; SpringIdx < CachedSuspensionInfo.Num(); SpringIdx++)
	{
		auto& Susp = CachedSuspensionInfo[SpringIdx];
		float NaturalFrequency = FSuspensionUtility::ComputeNaturalFrequency(CachedSuspensionInfo[SpringIdx].SuspensionData.SpringRate, OutSprungMasses[SpringIdx]);
		float Damping = FSuspensionUtility::ComputeDamping(CachedSuspensionInfo[SpringIdx].SuspensionData.SpringRate, OutSprungMasses[SpringIdx], CachedSuspensionInfo[SpringIdx].SuspensionData.DampingRatio);

		CachedSuspensionInfo[SpringIdx].ReboundDamping = Damping;
		CachedSuspensionInfo[SpringIdx].BoundDamping = Damping;
		CachedSuspensionInfo[SpringIdx].RestingForce = OutSprungMasses[SpringIdx] * -GravityGround;
	}

	// DEPRECATION AREA --
#if WITH_EDITOR
	ensureMsgf(false, TEXT("ScalarFrictionVal has been deprecated, please use LateralFrictionModifier and remove ScalarFrictionVal."));
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

	if (!SteeringActionCurve)
	{
		UE_LOG(LogTemp, Warning, TEXT("SteeringActionCurve missing, assuming default values."));
	}

	if (!AirControlCurve)
	{
		UE_LOG(LogTemp, Warning, TEXT("AirControlCurve missing, assuming default values."));
	}


#if WITH_EDITOR
	ensureMsgf(FMath::Max(FMath::Max(MaxSpeed, FMath::Abs(MaxBackwardsSpeed)), MaxSpeedBoosting) < TerminalSpeed, TEXT("Terminal Speed is lower than the max speed values."));
#endif
}
