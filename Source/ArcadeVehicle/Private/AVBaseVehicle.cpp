// Fill out your copyright notice in the Description page of Project Settings.


#include "AVBaseVehicle.h"
#include "CollisionQueryParams.h"
#include "PhysicalMaterials/PhysicalMaterial.h"

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
	PrimaryActorTick.bCanEverTick = true;

	// Custom Properties
	SuspensionFront = FSuspensionData();
	SuspensionRear = FSuspensionData();
	bIsMovingOnGround = false;
	bIsCloseToGround = false;
	bOverRollForceThreshold = false;
	GravityAir = -980.0;
	GravityGround = -980.0;
	// Gameplay driven friction
	ScalarFrictionVal = 1.f;
	GroundFriction = 100.f;
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
	bIsBoosting = false;
	CurrentThrottleAxis = 0.f;
	CurrentBrakeAxis = 0.f;
	bTiltedThrottle = true;
	VehicleAcceleration = 5000.f;
	VehicleBoostAcceleration = 3000.f;
	TorqueSpeed = 12.f;
	TimeFalling = 0.f;
	BrakinDeceleration = 2000.f;
	GroundDetectionDistanceThreshold = 800.f;
	AntiRollForcedDistanceThreshold = 100.f;
	AccelerationCenterOfMassOffset = FVector2D(50.f, 40.f);
	RGUpVector = FVector::ZeroVector;
	RGRightVector = FVector::ZeroVector;
	RGForwardVector = FVector::ZeroVector;
	CurrentGroundFriction = DEFAULT_GROUND_FRICTION;
	CurrentGroundScalarResistance = DEFAULT_GROUND_RESISTANCE;
	AccelerationAccumulatedTime = 0.f;
	LegalSpeedOffset = 100.f;
	bCompletelyInTheAir = false;
	bCompletelyInTheGround = true;
	AirYawSpeed = 20.f;
	AirStrafeSpeed = 1000.f;
	bOrientRotationToMovementInAir = false;
	OrientRotationToMovementInAirInfluenceRate = 1.f;
	SteeringDecelerationSideVelocityFactorScale = 40.f;
	SteeringDecelerationSideVelocityInterpolationSpeed = 0.4;


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
		static ConstructorHelpers::FObjectFinder<UStaticMesh> CylinderHelpMesh(TEXT("StaticMesh'/ArcadeVehicle/HelperCylinder.HelperCylinder'"));
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


/** Util for drawing result of single line trace  */
void DrawDebugLineTraceSingle(const UWorld* World, const FVector& Start, const FVector& End, EDrawDebugTrace::Type DrawDebugType, bool bHit, const FHitResult& OutHit, FLinearColor TraceColor, FLinearColor TraceHitColor, float DrawTime)
{
	if (DrawDebugType != EDrawDebugTrace::None)
	{
		bool bPersistent = DrawDebugType == EDrawDebugTrace::Persistent;
		float LifeTime = (DrawDebugType == EDrawDebugTrace::ForDuration) ? DrawTime : 0.f;

		// @fixme, draw line with thickness = 2.f?
		if (bHit && OutHit.bBlockingHit)
		{
			// Red up to the blocking hit, green thereafter
			::DrawDebugLine(World, Start, OutHit.ImpactPoint, TraceColor.ToFColor(true), bPersistent, LifeTime);
			::DrawDebugLine(World, OutHit.ImpactPoint, End, TraceHitColor.ToFColor(true), bPersistent, LifeTime);
			::DrawDebugPoint(World, OutHit.ImpactPoint, 16.f, TraceColor.ToFColor(true), bPersistent, LifeTime);
		}
		else
		{
			// no hit means all red
			::DrawDebugLine(World, Start, End, TraceColor.ToFColor(true), bPersistent, LifeTime);
		}
	}
}


// Called when the game starts or when spawned
void AAVBaseVehicle::BeginPlay()
{

	PawnRootComponent = Cast<UPrimitiveComponent>(GetRootComponent());
	if (PawnRootComponent != NULL) 
	{
		CachedSuspensionInfo.Init(FCachedSuspensionInfo(), NUMBER_OF_WHEELS);
		CachedSuspensionInfo[FRONT_LEFT].SuspensionData = SuspensionFront;
		CachedSuspensionInfo[FRONT_RIGHT].SuspensionData = SuspensionFront;
		CachedSuspensionInfo[BACK_LEFT].SuspensionData = SuspensionRear;
		CachedSuspensionInfo[BACK_RIGHT].SuspensionData = SuspensionRear;

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

		RootBodyInstance = PawnRootComponent->GetBodyInstance();
	}

	UWorld* World = GetWorld();
	if (World)
	{
		FPhysScene* PScene = World->GetPhysicsScene();
		if (PScene)
		{
			// Register physics step delegate
			OnPhysSceneStepHandle = PScene->OnPhysSceneStep.AddUObject(this, &AAVBaseVehicle::PhysSceneStep);
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
	LocalVelocity = RGWorldTransform.GetRotation().UnrotateVector(CurrentHorizontalVelocity);

	// Input processing
	ApplyInputStack(SubstepDeltaTime);


	ApplySuspensionForces();

	/************************************************************************/
	/* Apply gravity and sliding forces                                     */
	/************************************************************************/
	ApplyGravityForce(SubstepDeltaTime);

	if (bIsMovingOnGround)
	{
		// Drag/Friction force
		const FVector FrictionDragForce = bIsMovingOnGround ? ((FVector::DotProduct(RGRightVector, CurrentHorizontalVelocity) * -1.f) * (ScalarFrictionVal * GroundFriction * CurrentGroundFriction)) * RGRightVector : FVector::ZeroVector;
		RootBodyInstance->AddForce(FrictionDragForce, false, false);
	}
	
	
	/************************************************************************/
	/* End apply gravity and sliding forces									*/
	/************************************************************************/

	// Throttle force adjustments based on Brake and forward axis values
	//if (!bIsBoosting)
	//{
		const FVector side_velocity = FMath::Abs(LocalVelocity.Y) * RGForwardVector;
		const FVector non_forward_velocity = CurrentHorizontalVelocity.GetSafeNormal2D() * getMaxSpeed() - (side_velocity * SteeringDecelerationSideVelocityFactorScale);
		const bool bTurnDeceleration = CurrentHorizontalSpeed > non_forward_velocity.Size() && !bIsBoosting && !FMath::IsNearlyZero(CurrentSteeringAxis);

		if (bTurnDeceleration)
		{
			ThrottleForce = (non_forward_velocity.GetSafeNormal2D() * -CurrentHorizontalSpeed * SteeringDecelerationSideVelocityInterpolationSpeed);
		}
		
	
		const float SpeedRatio = CurrentHorizontalSpeed / GetAbsMaxSpeedAxisIndependent();
		const float AxisAdjustment = CurrentBrakeAxis + CurrentThrottleAxis;
		const bool bNoInput = (CurrentBrakeAxis > -0.1 && CurrentThrottleAxis < 0.1);
		
		// Clamping curve thresholds
		const float AccumulativeAxisAccel = FMath::Clamp(AccelerationAccumulatedTime, MinAccelerationCurveTime, MaxAccelerationCurveTime);
		AccelerationAccumulatedTime = (FMath::IsNearlyZero(AxisAdjustment) || bTurnDeceleration) ? FMath::GetMappedRangeValueClamped(FVector2D(-1, 1), FVector2D(MinAccelerationCurveTime, MaxAccelerationCurveTime), SpeedRatio) : AccumulativeAxisAccel;
		const float AccumulativeAxisDecel = DecelerationAccumulatedTime > MaxDecelerationCurveTime ? MaxDecelerationCurveTime : DecelerationAccumulatedTime + SubstepDeltaTime;
		DecelerationAccumulatedTime = bNoInput ? AccumulativeAxisDecel : FMath::GetMappedRangeValueClamped(FVector2D(0, 1), FVector2D(MaxDecelerationCurveTime, 0), FMath::Abs(SpeedRatio));
		
		PRINT_TICK(FString::SanitizeFloat(AccelerationAccumulatedTime));
		PRINT_TICK(FString::SanitizeFloat(DecelerationAccumulatedTime));

		// Engine deceleration only if no input is applied whatsoever
		const FVector ThrottleAdjustmentRatio = CurrentHorizontalVelocity * bNoInput * GetDecelerationRatio();
		
		// Forcing vehicle decceleration if they surpass the LegalSpeedOffset
		const bool bIsGoingOverLegalSpeed = (FMath::Abs(CurrentHorizontalSpeed) > ((GetAbsMaxSpeedAxisIndependent() * CurrentGroundScalarResistance) + LegalSpeedOffset));
		ThrottleForce = bIsGoingOverLegalSpeed ? FVector::VectorPlaneProject(-CurrentHorizontalVelocity.GetSafeNormal(), AvgedNormals) * getAcceleration() : ThrottleForce;

		// Adjusting Throttle based on engine decceleration values
		ThrottleForce = bIsMovingOnGround ? ThrottleForce - ThrottleAdjustmentRatio : ThrottleForce;
	//}
	
	

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
	ThrottleForce = FVector::ZeroVector;
	SteeringForce = FVector::ZeroVector;
}


FSuspensionHitInfo AAVBaseVehicle::CalcSuspension(FVector RelativeOffset, FCachedSuspensionInfo& InCachedInfo)
{
	const FVector TraceStart = RGWorldTransform.TransformPositionNoScale(RelativeOffset);
	const FVector TraceEnd = TraceStart - (RGUpVector * (GroundDetectionDistanceThreshold));
	FSuspensionHitInfo sHitInfo;

	FHitResult OutHit;
	InCachedInfo.SuspensionRatio = 0.f;
	
	if (TraceFunc(TraceStart, TraceEnd, bDebugInfo > 0 ? EDrawDebugTrace::ForOneFrame : EDrawDebugTrace::None, OutHit))
	{
		InCachedInfo.SuspensionRatio = FMath::Max(0.f, 1 - (OutHit.Distance / InCachedInfo.SuspensionData.SuspensionLength));

		if (InCachedInfo.SuspensionRatio > 0.f)
		{
			const FVector VectorToProject = RootBodyInstance->GetUnrealWorldVelocityAtPoint(TraceStart) * InCachedInfo.SuspensionData.SuspensionStiffness;
			const FVector ForceDownwards = AvgedNormals.SizeSquared() > SMALL_NUMBER ? VectorToProject.ProjectOnTo(AvgedNormals) : FVector::ZeroVector;
			const FVector FinalForce = (AvgedNormals * (InCachedInfo.SuspensionRatio * InCachedInfo.SuspensionData.SuspensionDampForce)) - ForceDownwards;
			RootBodyInstance->AddForceAtPosition(FinalForce, TraceStart, false);


			#if ENABLE_DRAW_DEBUG
			if (bDebugInfo)
			{
				UWorld* World = GEngine->GetWorldFromContextObject(this, EGetWorldErrorMode::LogAndReturnNull);
				if (World) UKismetSystemLibrary::DrawDebugString(World, TraceStart, FString::SanitizeFloat(InCachedInfo.SuspensionRatio).Mid(0, 4), nullptr, FLinearColor::White, 0.f);
			}
			#endif

			// Trace hits and it is within the suspension length size, hence the wheel is on the ground
			sHitInfo.bWheelOnGround = true;

			// Physical material properties for this wheel contacting the ground
			const bool bPhysMatExists = OutHit.PhysMaterial.IsValid();
			sHitInfo.GroundFriction = bPhysMatExists ? OutHit.PhysMaterial->Friction : DEFAULT_GROUND_FRICTION;
			sHitInfo.GroundResistance = bPhysMatExists ? OutHit.PhysMaterial->Density : DEFAULT_GROUND_RESISTANCE;
		}		
		sHitInfo.bTraceHit = true;
		sHitInfo.bOverRollForceThreshold = OutHit.Distance > AntiRollForcedDistanceThreshold;

		InCachedInfo.ImpactNormal = OutHit.ImpactNormal;
		InCachedInfo.ImpactPoint = OutHit.ImpactPoint;
		return sHitInfo;
	}

	return sHitInfo;
}


void AAVBaseVehicle::ApplySuspensionForces()
{
	
	const FSuspensionHitInfo BackRightSuspension = CalcSuspension(BackRight, CachedSuspensionInfo[BACK_RIGHT]);
	const FSuspensionHitInfo FrontRightSuspension = CalcSuspension(FrontRight, CachedSuspensionInfo[FRONT_RIGHT]);
	const FSuspensionHitInfo FrontLeftSuspension = CalcSuspension(FrontLeft, CachedSuspensionInfo[FRONT_LEFT]);
	const FSuspensionHitInfo BackLeftSuspension = CalcSuspension(BackLeft, CachedSuspensionInfo[BACK_LEFT]);
	
	// On landing impl. If in the previous step it was in the air and now is in the ground 
	const bool bWasMovingOnGround = bIsMovingOnGround;

	const uint8 WheelsTouchingTheGround = (BackRightSuspension.bWheelOnGround + FrontRightSuspension.bWheelOnGround + FrontLeftSuspension.bWheelOnGround + BackLeftSuspension.bWheelOnGround);
	bIsMovingOnGround = WheelsTouchingTheGround > 2;
	bCompletelyInTheAir = WheelsTouchingTheGround == 0;
	bCompletelyInTheGround = WheelsTouchingTheGround == NUMBER_OF_WHEELS;
	bOverRollForceThreshold = (BackRightSuspension.bOverRollForceThreshold + FrontRightSuspension.bOverRollForceThreshold + FrontLeftSuspension.bOverRollForceThreshold + BackLeftSuspension.bOverRollForceThreshold) > 2;
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


bool AAVBaseVehicle::TraceFunc_Implementation(FVector Start, FVector End, EDrawDebugTrace::Type DrawDebugType, FHitResult& OutHit)
 {
	FHitResult Hit(ForceInit);
	static const FName HoverLineTraceName(TEXT("HoverTrace"));
	FCollisionQueryParams TraceParams;
	TraceParams.TraceTag = HoverLineTraceName;
	TraceParams.bTraceComplex = false;
	TraceParams.bReturnPhysicalMaterial = true;
	TraceParams.AddIgnoredActor(this);
	bool const bHit = GetWorld()->LineTraceSingleByChannel(OutHit, Start, End, ECC_Visibility, TraceParams);
#if ENABLE_DRAW_DEBUG
	DrawDebugLineTraceSingle(GetWorld(), Start, End, DrawDebugType, bHit, OutHit, FColor::Red, FColor::Green, 5.f);
#endif
	return bHit;
}


FVector AAVBaseVehicle::GetOffsetedCenterOfVehicle() const
{
	return (RGLocation - (RGUpVector * AccelerationCenterOfMassOffset.Y)) +
		((RGForwardVector * (CurrentThrottleAxis + CurrentBrakeAxis)) * AccelerationCenterOfMassOffset.X);
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
	// Throttle
	if (CurrentThrottleAxis >= 0.1f || bIsBoosting)
	{
		AccelerationAccumulatedTime += DeltaTime;
		/** We don't decelerate when we go over max speed, since we want vehicles to gain velocity downhill
		* The terminal will be the maximum individual top speed that any vehicle can reach in any situation
		**/
		if (CurrentHorizontalSpeed <= GetComputedSpeed() && bIsMovingOnGround)
		{
			const float MaxThrottleRatio = bIsBoosting ? 1.f : CurrentThrottleAxis;
			ThrottleForce =  FVector::VectorPlaneProject(RGForwardVector, AvgedNormals) * getAcceleration() * MaxThrottleRatio;
		}
	}

	// Steering
	if ((CurrentSteeringAxis >= 0.1f && CurrentAngularSpeed <= TorqueSpeed) ||
		(CurrentSteeringAxis <= -0.1f && CurrentAngularSpeed >= -TorqueSpeed))
	{
		// fixmevori: simplify
		// Direction Calc
		const float DirectionSign = FMath::Sign(CurrentHorizontalSpeed);
		const float DirectionFactor = (DirectionSign == 0 || (CurrentHorizontalSpeed <= 0 && CurrentHorizontalSpeed > -100 && !bIsMovingOnGround) ) ? 1.f : DirectionSign;
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
				const float forward_signed_speed = (RGForwardVector | CurrentHorizontalVelocity);
				const FVector forward_velocity = (RGForwardVector * forward_signed_speed * DirectionFactor);
				const FVector non_forward_velocity = CurrentHorizontalVelocity - forward_velocity;
				//if (non_forward_velocity.SizeSquared2D() > 1000.f) {
					ThrottleForce = non_forward_velocity.GetSafeNormal2D() * -CurrentHorizontalSpeed * OrientRotationToMovementInAirInfluenceRate * ORIENT_ROTATION_VELOCITY_MAX_RATE;
				//}

				

				// fixmevori: Experimental code, might change to drag and lift force model if i get it working. :)
				/*
				float fMyAwsomeDot = 1.0f - (FVector::DotProduct(CurrentHorizontalVelocity.GetSafeNormal2D(), RGForwardVector.GetSafeNormal2D()) * 0.5f + 0.5f);
				fMyAwsomeDot = FMath::Pow(fMyAwsomeDot, 1);
				fMyAwsomeDot *= CurrentHorizontalVelocity.Size();
				fMyAwsomeDot *= 5 * fMyAwsomeDot;
				FVector vDragAndLiftForce = -fMyAwsomeDot * CurrentHorizontalVelocity + fMyAwsomeDot * RGForwardVector;
				ThrottleForce = vDragAndLiftForce;
				*/
			}
			else
			{
				ThrottleForce = CurrentSteeringAxis * RGRightVector * AirStrafeSpeed * ControlRatio;
			}
		}
		
		SteeringForce = RGUpVector * DirectionFactor * InputTorqueRatio;
	}

	// Braking
	if (CurrentBrakeAxis <= -0.1 && !bIsBoosting)
	{
		//if (CurrentHorizontalSpeed <= 0.0)
		//{
			AccelerationAccumulatedTime -= DeltaTime;
			if (CurrentHorizontalSpeed > GetComputedSpeed() && bIsMovingOnGround)
			{
				ThrottleForce += FVector::VectorPlaneProject(RGForwardVector, AvgedNormals) * CurrentBrakeAxis * getAcceleration();
			}
		/*}
		else
		{
			ThrottleForce += FVector::VectorPlaneProject(RGForwardVector, AvgedNormals) * CurrentBrakeAxis * BrakinDeceleration;
		}*/
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
	FVector GravityForce = FVector(0.f, 0.f, bIsMovingOnGround ? GravityGround : GravityAir);
	FVector CorrectionalUpVectorFlippingForce = FVector::UpVector;

	if (bStickyWheels && bIsCloseToGround && !AvgedNormals.IsZero()) 
	{
		GravityForce = AvgedNormals * (bIsMovingOnGround ? GravityGround : GravityAir);
		CorrectionalUpVectorFlippingForce = AvgedNormals;	
	}

	// Air vectors
	if (bCompletelyInTheAir)
	{
		TimeFalling += DeltaTime;

		const float DotProductUpvectors = FVector::DotProduct(RGUpVector, CorrectionalUpVectorFlippingForce);
		const float MappedDotProduct = FMath::GetMappedRangeValueClamped(FVector2D(-1.f, 1.f), FVector2D(1, 0.f), DotProductUpvectors);
		
		// Anti roll force (the car should be straight!)
		const FVector AntiRollForce = bOverRollForceThreshold ? FVector::CrossProduct(CorrectionalUpVectorFlippingForce, -RGUpVector) * FMath::Lerp(200.f, 1000.f, MappedDotProduct) : FVector::ZeroVector;
		const FVector AngularFinalForce = (AntiRollForce + SteeringForce) * DeltaTime;
		RootBodyInstance->SetAngularVelocityInRadians(AngularFinalForce, false);
	}

	RootBodyInstance->AddForce(GravityForce, false, false);
}


float AAVBaseVehicle::GetTerminalSpeed() const
{
	return TerminalSpeed;
}


bool AAVBaseVehicle::IsBraking() const
{
	return ((CurrentBrakeAxis <= -0.1 && CurrentHorizontalSpeed > 0.f) || 
		(CurrentThrottleAxis >= 0.1 && CurrentHorizontalSpeed <= 0.f) ||
		(CurrentThrottleAxis + CurrentBrakeAxis) == 0);
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
			|| PropertyThatChanged->GetFName() == GET_MEMBER_NAME_CHECKED(AAVBaseVehicle, SuspensionRear.SuspensionLength))
		{
			BackRightHandle->SetRelativeScale3D( FVector(0.05f, 0.05f, SuspensionRear.SuspensionLength / 100.f));
			FrontRightHandle->SetRelativeScale3D( FVector(0.05f, 0.05f, SuspensionFront.SuspensionLength / 100.f));
			FrontLeftHandle->SetRelativeScale3D( FVector(0.05f, 0.05f, SuspensionFront.SuspensionLength / 100.f));
			BackLeftHandle->SetRelativeScale3D( FVector(0.05f, 0.05f, SuspensionRear.SuspensionLength / 100.f));
		}
		
		if (PropertyThatChanged->GetFName() == GET_MEMBER_NAME_CHECKED(AAVBaseVehicle, bHideHelpHandlersInPIE))
		{
			BackRightHandle->SetHiddenInGame(bHideHelpHandlersInPIE);
			FrontRightHandle->SetHiddenInGame(bHideHelpHandlersInPIE);
			FrontLeftHandle->SetHiddenInGame(bHideHelpHandlersInPIE);
			BackLeftHandle->SetHiddenInGame(bHideHelpHandlersInPIE);
		}

		if (PropertyThatChanged->GetFName() == GET_MEMBER_NAME_CHECKED(AAVBaseVehicle, GroundDetectionDistanceThreshold)
			|| PropertyThatChanged->GetFName() == GET_MEMBER_NAME_CHECKED(AAVBaseVehicle, AntiRollForcedDistanceThreshold))
		{
			AntiRollForcedDistanceThreshold = FMath::Min(GroundDetectionDistanceThreshold, AntiRollForcedDistanceThreshold);
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
	BackRightHandle->SetRelativeScale3D(FVector(0.05f, 0.05f, SuspensionRear.SuspensionLength / 100.f));
	FrontRightHandle->SetRelativeScale3D(FVector(0.05f, 0.05f, SuspensionFront.SuspensionLength / 100.f));
	FrontLeftHandle->SetRelativeScale3D(FVector(0.05f, 0.05f, SuspensionFront.SuspensionLength / 100.f));
	BackLeftHandle->SetRelativeScale3D(FVector(0.05f, 0.05f, SuspensionRear.SuspensionLength / 100.f));

	// We don't allow the user to rotate the handlers
	BackRightHandle->SetRelativeRotation(FRotator::ZeroRotator);
	FrontRightHandle->SetRelativeRotation(FRotator::ZeroRotator);
	FrontLeftHandle->SetRelativeRotation(FRotator::ZeroRotator);
	BackLeftHandle->SetRelativeRotation(FRotator::ZeroRotator);
}


void AAVBaseVehicle::UpdateHandlersTransformCDO()
{
	BackRightHandle->SetRelativeTransform(FTransform(FRotator::ZeroRotator, BackRight, FVector(0.05f, 0.05f, SuspensionRear.SuspensionLength / 100.f)));
	FrontRightHandle->SetRelativeTransform(FTransform(FRotator::ZeroRotator, FrontRight, FVector(0.05f, 0.05f, SuspensionFront.SuspensionLength / 100.f)));
	FrontLeftHandle->SetRelativeTransform(FTransform(FRotator::ZeroRotator, FrontLeft, FVector(0.05f, 0.05f, SuspensionFront.SuspensionLength / 100.f)));
	BackLeftHandle->SetRelativeTransform(FTransform(FRotator::ZeroRotator, BackLeft, FVector(0.05f, 0.05f, SuspensionRear.SuspensionLength / 100.f)));
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