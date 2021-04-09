// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "Kismet/KismetSystemLibrary.h"
#include "Engine/Public/DrawDebugHelpers.h"
#include "AVBaseVehicle.generated.h"

// Suspension

#define FRONT_LEFT 0
#define FRONT_RIGHT 1
#define BACK_LEFT 2
#define BACK_RIGHT 3
#define NUMBER_OF_WHEELS 4
#define NUMBER_OF_AXLES 2


// Misc vars
#define DEFAULT_GROUND_FRICTION 1
#define DEFAULT_GROUND_RESISTANCE 1
#define TERMINAL_VELOCITY_PREEMPTION_FORCE_OFFSET 2000.f
#define IDLE_VEHICLE_FORCE 500.f
#define ORIENT_ROTATION_VELOCITY_MAX_RATE 10.f
#define BASE_GROUND_FRICTION 100


#define PRINT_TICK(x) UKismetSystemLibrary::PrintString(this,x,true,false,FLinearColor::Red, 0.f)
#define PRINT_TICK_LOG(x) UKismetSystemLibrary::PrintString(this,x,true,true,FLinearColor::Red, 0.f)


#if WITH_EDITOR
class UArrowComponent;
#endif

USTRUCT()
struct ARCADEVEHICLE_API FSuspensionHitInfo
{
	GENERATED_USTRUCT_BODY()

public:

	UPROPERTY()
	uint8 bWheelOnGround : 1;

	UPROPERTY()
	uint8 bTraceHit : 1;

	UPROPERTY()
	float GroundFriction;

	UPROPERTY()
	float GroundResistance;

	UPROPERTY()
	float SusForce;

	UPROPERTY()
	FVector WheelRestingWorldLocation;

	FSuspensionHitInfo() :
		bWheelOnGround(false),
		bTraceHit(false),
		GroundFriction(DEFAULT_GROUND_FRICTION),
		GroundResistance(DEFAULT_GROUND_RESISTANCE),
		SusForce(0.f),
		WheelRestingWorldLocation(FVector::ZeroVector)
	{

	}
};

USTRUCT(BlueprintType)
struct ARCADEVEHICLE_API FSuspensionData
{
	GENERATED_USTRUCT_BODY()

public:

	/** How far the wheel can go above the resting position */
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, meta = (ClampMin = "0.0", UIMin = "0.0"))
	float SuspensionMaxRaise;

	/** How far the wheel can drop below the resting position */
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, meta = (ClampMin = "0.0", UIMin = "0.0"))
	float SuspensionMaxDrop;

	/** Spring Force */
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, meta = (ClampMin = "0.0", UIMin = "0.0"))
	float SpringRate;

	/** Spring damping: (0 - no damping) and (1 - critical damping) */
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, meta = (ClampMin = "0.0", UIMin = "0.0", ClampMax = "1.0", UIMax = "1.0"))
	float DampingRatio;

	/**
	*	When 0 no weight load is transferred, 1 is Normal weight shift. Lower value cures lift off oversteer. Requires bComputeAxleForces = true
	*/
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, meta = (ClampMin = "0.0", UIMin = "0.0", ClampMax = "1.0", UIMax = "1.0"))
	float SuspensionLoadRatio;

	/* Radius of the wheel */
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, meta = (ClampMin = "0.0", UIMin = "0.0"))
	float WheelRadius;

	/* Trace Half Size (wheel dimensions) */
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly)
	FVector2D TraceHalfSize;

	FSuspensionData() :
		SuspensionMaxRaise(20.f),
		SuspensionMaxDrop(12.f),
		SpringRate(50.f),
		DampingRatio(0.5f),
		SuspensionLoadRatio(0.5f),
		WheelRadius(20.f),
		TraceHalfSize(30.f, 30.f)
	{

	}

	FSuspensionData(float inSuspensionMaxRaise, float inSuspensionMaxDrop, float inSpringRate, float inDampingRatio, float inSuspensionLoadRatio, float inWheelRadius, FVector2D inTraceHalfSize) :
		SuspensionMaxRaise(inSuspensionMaxRaise),
		SuspensionMaxDrop(inSuspensionMaxDrop),
		SpringRate(inSpringRate),
		DampingRatio(inDampingRatio),
		SuspensionLoadRatio(inSuspensionLoadRatio),
		WheelRadius(inWheelRadius),
		TraceHalfSize(inTraceHalfSize)
	{

	}

};

USTRUCT(BlueprintType)
struct ARCADEVEHICLE_API FCachedSuspensionInfo
{
	GENERATED_USTRUCT_BODY()

public:

	UPROPERTY()
	FVector ImpactNormal;

	UPROPERTY()
	FVector WheelRelativeLocation;

	UPROPERTY(BlueprintReadOnly)
	FSuspensionData SuspensionData;

	UPROPERTY(BlueprintReadOnly)
	float BoundDamping;

	UPROPERTY(BlueprintReadOnly)
	float ReboundDamping;

	UPROPERTY(BlueprintReadOnly)
	float RestingForce;

	UPROPERTY(BlueprintReadOnly)
	float DisplacementInput;

	UPROPERTY(BlueprintReadOnly)
	float LastDisplacement;

	UPROPERTY(BlueprintReadOnly)
	float SuspensionLength;

	FCachedSuspensionInfo() :
		ImpactNormal(FVector::ZeroVector),
		WheelRelativeLocation(FVector::ZeroVector),
		SuspensionData(FSuspensionData()),
		BoundDamping(0.f),
		ReboundDamping(0.f),
		RestingForce(0.f),
		DisplacementInput(0.f),
		LastDisplacement(0.f),
		SuspensionLength(0.f)
	{

	}
};


USTRUCT()
struct ARCADEVEHICLE_API FBasedPlatformInfo
{
	GENERATED_USTRUCT_BODY()
	
	/** Component we are based on */
	UPROPERTY()
	UPrimitiveComponent* MovementBase;

	/** Location relative to MovementBase. */
	UPROPERTY()
	FVector_NetQuantize100 Location;

	/** Rotation relative to MovementBase. */
	UPROPERTY()
	FQuat Rotation;

};


USTRUCT(BlueprintType)
struct ARCADEVEHICLE_API FWheelAnimationData
{
	GENERATED_USTRUCT_BODY()

public:

	UPROPERTY()
	FVector2D WheelSpinningSpeed;
	
	UPROPERTY()
	FVector2D WheelAngularPosition;
	
	UPROPERTY()
	FVector2D WheelCurrentSteer;

	UPROPERTY(EditDefaultsOnly)
	FVector2D WheelMaxSteerAngle;
	
	FWheelAnimationData() :
		WheelSpinningSpeed(FVector2D::ZeroVector),
		WheelAngularPosition(FVector2D::ZeroVector),
		WheelCurrentSteer(FVector2D::ZeroVector),
		WheelMaxSteerAngle(FVector2D(25.f, 0.f))
	{

	}
};


UENUM(BlueprintType)
enum class EBasedPlatformSetup : uint8 {
	IgnoreBasedMovement		= 0		UMETA(DisplayName = "Ignore Based Movement"),
	IgnoreBasedRotation		= 1		UMETA(DisplayName = "Ignore Based Rotation"),
	ApplyBasedMovement		= 2		UMETA(DisplayName = "Apply Based Movement"),
};


UENUM(BlueprintType)
enum class EAirNavigationMode : uint8 {
	None = 0					UMETA(DisplayName = "None"),
	Predictive = 1				UMETA(DisplayName = "Ballistic Prediction"),
	GroundAdaptative = 2		UMETA(DisplayName = "Ground Adaptative"),
};


UENUM(BlueprintType)
enum class ESimplifiedDirection : uint8 {
	Idle	= 0		UMETA(DisplayName = "Idle"),
	Forward = 1		UMETA(DisplayName = "Forward Movement"),
	Reverse = 2		UMETA(DisplayName = "Backwards Movement"),
	Right	= 3		UMETA(DisplayName = "Right Movement"),
	Left	= 4		UMETA(DisplayName = "Left Movement"),
};


UCLASS()
class ARCADEVEHICLE_API AAVBaseVehicle : public APawn
{
	GENERATED_BODY()

public:
	// Sets default values for this pawn's properties
	AAVBaseVehicle();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;


public:	


	// Called to bind functionality to input
	virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

public:

	/* Initialises the vehicle */
	virtual void InitVehicle();

	/* Prepares the different curves of the vehicle */
	void SetupVehicleCurves();

	FDelegateHandle OnPhysSceneStepHandle;
	void PhysSceneStep(FPhysScene* PhysScene, float DeltaTime);

	/* This event is called on every physics tick, including sub-steps. */
	void PhysicsTick(float SubstepDeltaTime);

	/**
	/*	Computes the force magnitude for a suspension point
	**/
	UFUNCTION(BlueprintPure)
	virtual float GetSuspensionForceMagnitude(const FCachedSuspensionInfo& InCachedInfo, float DeltaTime) const;

	/**
	/*	Computes the vehicle drag force
	**/
	UFUNCTION(BlueprintPure)
	virtual FVector GetLateralFrictionDragForce() const;

	/**
	/*	Unifies the calculation of all the suspension points.
	**/
	UFUNCTION()
	virtual void ApplySuspensionForces(float DeltaTime);

	/**
	/*	Computes wheel rotation and steering
	**/
	UFUNCTION()
	virtual void SimulateWheelMovement(float DeltaTime);

	/**
	/*	Tracing function employed to compute the suspensions.
	**/
	UFUNCTION(BlueprintNativeEvent, meta = (DisplayName = "TraceFunc"))
	bool TraceFunc(FVector Start, FVector End, FVector2D HalfSize, EDrawDebugTrace::Type DrawDebugType, FHitResult& OutHit);

	/**
	/*	Returns the current center of mass of the vehicle.
	**/
	UFUNCTION(BlueprintPure)
	FVector GetOffsetedCenterOfVehicle() const;

	UFUNCTION()
	void SetThrottleInput(float InputAxis);

	UFUNCTION()
	void SetSteeringInput(float InputAxis);

	UFUNCTION()
	void SetBrakeInput(float InputAxis);

	void ApplyInputStack(float DeltaTime);

	UFUNCTION()
	virtual void ApplySteeringInput(float DeltaTime);

	UFUNCTION()
	virtual void ApplyThrottleInput(float DeltaTime);

	UFUNCTION()
	virtual void ApplyReverseInput(float DeltaTime);

	UFUNCTION()
	virtual void ApplyBrakeInput(float DeltaTime);

	/** Gets you whatever max speed is being used in absolute value */
	UFUNCTION(BlueprintPure)
	float GetAbsMaxSpeedAxisIndependent() const;

	/** Computes the current forward speed based on the acceleration curve */
	UFUNCTION(BlueprintPure)
	float GetComputedSpeed() const;

	UFUNCTION(BlueprintPure)
	float GetDecelerationRatio() const;

	UFUNCTION(BlueprintPure)
	float getMaxSpeed() const;

	UFUNCTION(BlueprintPure)
	float getMaxBackwardsSpeed() const;

	UFUNCTION(BlueprintPure)
	float getAcceleration() const;

	UFUNCTION(BlueprintCallable)
	void SetBoosting(bool inBoost);

	/* Call this function whenever you want to initialise the vehicle (ie: after a recover from out of bounds... ect) */
	UFUNCTION(BlueprintCallable)
	virtual void ResetVehicle();

	UFUNCTION(BlueprintPure)
	bool GetStickyWheels() const;

	UFUNCTION(BlueprintCallable)
	void SetStickyWheels(bool inStickyWheels);

	UFUNCTION(BlueprintCallable)
	virtual void ApplyGravityForce(float DeltaTime);

	UFUNCTION(BlueprintPure)
	float GetTerminalSpeed() const;

	/** Is any brakin force acting over the vehicle? */
	UFUNCTION(BlueprintPure)
	bool IsBraking() const;

	/** Maps the vehicle direction to a simple enum easy to read */
	UFUNCTION(BlueprintPure)
	ESimplifiedDirection GetSimplifiedKartDirection() const;

	/** This function gets called when bIsMovingOnGround becomes true */
	virtual void Landed(const FVector& HitNormal);

	UFUNCTION(BlueprintImplementableEvent)
	void OnLanded(const FVector &HitNormal);

	/** Computes the delta movement to apply to the vehicle on moving platforms **/
	void ComputeBasedMovement();

	/** Computes the ballistic trajectory to apply the vehicle in the Ballistic air navigation mode. */
	bool KartBallisticPrediction(
		FVector StartPos,
		FVector LaunchVelocity,
		TEnumAsByte<ECollisionChannel> TraceChannel,
		float SimFrequency,
		float MaxSimTime,
		FVector& OutNormal);


	/** Calc Suspension function used for simulated proxies	*/
	FVector CalcSuspensionSimulatedProxy(FVector RelativeOffset, const FSuspensionData& SuspensionData);

	/** Handle to compute wheel ik
		- Simulated proxy: VFX + IK
		- Owning Client: IK
		X: Left wheel
		Y: Right wheel
	*/
	UFUNCTION(BlueprintCallable)
	void WheelsVisuals(FVector& FR, FRotator& FRR, FVector& FL, FRotator& FLR, FVector& RR, FRotator& RRR, FVector& RL, FRotator& RLR);

protected:

	FBodyInstance* RootBodyInstance;
	static FBodyInstance* GetBodyInstance(UPrimitiveComponent* PrimitiveComponent);

	UPROPERTY(BlueprintReadOnly)
	FVector CurrentHorizontalVelocity;

	UPROPERTY(BlueprintReadOnly)
	FVector LocalVelocity;
	
	/** 3D Speed scalar, the sign is based on whether the vehicle is navigating forwards or backwards */
	UPROPERTY()
	float CachedSpeedSized;

	UPROPERTY(BlueprintReadOnly)
	FVector CurrentAngularVelocity;

	UPROPERTY(BlueprintReadOnly)
	float CurrentAngularSpeed;

	/* Gameplay driven friction value to compute ground lateral drag friction. Possible use case: drifting. */
	UPROPERTY(BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float LateralFrictionModifier;

	UPROPERTY(BlueprintReadOnly)
	FVector AvgedNormals;

	/* It's true if at least two wheels are touching the ground */
	UPROPERTY(BlueprintReadOnly)
	uint8 bIsMovingOnGround : 1;

	/* It's true if any wheel is touching the ground */
	UPROPERTY(BlueprintReadOnly)
	uint8 bCompletelyInTheAir : 1;

	UPROPERTY(BlueprintReadOnly)
	uint8 bCompletelyInTheGround : 1;

	UPROPERTY(BlueprintReadOnly)
	uint8 bIsCloseToGround : 1;

	UPROPERTY(BlueprintReadWrite)
	float CurrentThrottleAxis;

	UPROPERTY(BlueprintReadWrite)
	float CurrentSteeringAxis;

	UPROPERTY(BlueprintReadWrite)
	float CurrentBrakeAxis;

	UPROPERTY(BlueprintReadWrite)
	FVector ThrottleForce;

	UPROPERTY(BlueprintReadWrite)
	FVector SteeringForce;

	UPROPERTY(BlueprintReadOnly)
	FVector RGForwardVector;

	UPROPERTY(BlueprintReadOnly)
	FVector RGUpVector;

	UPROPERTY(BlueprintReadOnly)
	FVector RGRightVector;

	UPROPERTY(BlueprintReadOnly)
	FVector RGLocation;

	UPROPERTY(BlueprintReadOnly)
	FTransform RGWorldTransform;

	UPROPERTY(BlueprintReadOnly)
	bool bIsBoosting;

	UPROPERTY(BlueprintReadOnly)
	float CurrentGroundFriction;

	/** Desired input direction **/
	UPROPERTY(BlueprintReadOnly)
	float InfluencialDirection;

	/** Desired input direction **/
	UPROPERTY(BlueprintReadOnly)
	uint8 bIsGearReady : 1;

	/** Resistance imposed by the current ground in which the user is navigating, will be translated to ground linear damping. */
	UPROPERTY(BlueprintReadWrite)
	float CurrentGroundScalarResistance;

	UPROPERTY(BlueprintReadOnly)
	float AccelerationAccumulatedTime;

	UPROPERTY(BlueprintReadOnly)
	float MinAccelerationCurveTime;

	UPROPERTY(BlueprintReadOnly)
	float MaxAccelerationCurveTime;

	UPROPERTY(BlueprintReadOnly)
	float DecelerationAccumulatedTime;

	UPROPERTY(BlueprintReadOnly)
	float MaxDecelerationCurveTime;

	UPROPERTY(BlueprintReadWrite)
	float MaxSpeed;

	UPROPERTY(BlueprintReadWrite)
	float MaxBackwardsSpeed;

	UPROPERTY(BlueprintReadOnly)
	float TimeFalling;
	
	UPROPERTY()
	FBasedPlatformInfo BasedPlatformInfo;

	UPROPERTY(BlueprintReadOnly)
	ESimplifiedDirection LastUsedGear;

	UPROPERTY(BlueprintReadOnly)
	ESimplifiedDirection LastUsedGroundGear;

	UPROPERTY(BlueprintReadOnly)
	float LastTimeGearSwapped;

	/* Is the vehicle being flipped? */
	UPROPERTY(BlueprintReadOnly)
	bool bFlipping;

	/* Animation data for the wheels */
	UPROPERTY(EditDefaultsOnly)
	FWheelAnimationData WheelAnimData;

	/** 1: Max influence 0: Min Influence **/
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, meta = (EditCondition = "bOrientRotationToMovementInAir", ClampMin = "0.0", UIMin = "0.0", ClampMax = "1.0", UIMax = "1.0"))
	float OrientRotationToMovementInAirInfluenceRate;

	/** How much the current Throttle affects the braking. 0: Min Influence, 1: Max influence */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	float AccelerationInfluenceRateWhileBraking;

	/** If true, the vehicle's gravity is the closest ground. If no closest ground is detected, -Z is employed. */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	bool bStickyWheels;

	/** Computes axle forces: Requires high precision. Works at 100 Hz. */
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly)
	bool bComputeAxleForces;

public:

#if WITH_EDITOR

	UPROPERTY(Category = HelperHandler, EditDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	UStaticMeshComponent* BackRightHandle;

	UPROPERTY(Category = HelperHandler, EditDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	UStaticMeshComponent* FrontRightHandle;

	UPROPERTY(Category = HelperHandler, EditDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	UStaticMeshComponent* FrontLeftHandle;

	UPROPERTY(Category = HelperHandler, EditDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	UStaticMeshComponent* BackLeftHandle;

	UPROPERTY(Category = HoverComponent, EditDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	bool bHideHelpHandlersInPIE;

#endif

	UPROPERTY(Category = HoverComponent, EditDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	UStaticMeshComponent* CollisionMesh;

	// Hoover/Wheel resting point position
	UPROPERTY(BlueprintReadOnly)
	FVector BackRight;

	UPROPERTY(BlueprintReadOnly)
	FVector FrontRight;

	UPROPERTY(BlueprintReadOnly)
	FVector FrontLeft;

	UPROPERTY(BlueprintReadOnly)
	FVector BackLeft;

	UPROPERTY(EditDefaultsOnly, Category = SuspensionFront)
	FSuspensionData SuspensionFront;

	UPROPERTY(EditDefaultsOnly, Category = SuspensionRear)
	FSuspensionData SuspensionRear;

	/* Decides how the vehicle behaves on top of moving or rotating platforms. */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	EBasedPlatformSetup BasedMovementSetup;

	/* Decides how the vehicle behaves while air navigating. */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	EAirNavigationMode AirNavigationMode;

	/** "Down" force to apply while in the ground. */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	float GravityAir;

	/** "Down" force to apply while in the air. */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	float GravityGround;

	/** How far our wheel traces go down to detect close terrains for sticky wheels and ground snapping navigation. */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	float GroundDetectionDistanceThreshold;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	float LinearDampingAir;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	float AngularDamping;

	/** Target speed when boosting. */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	float MaxSpeedBoosting;

	/** Acceleration curve (Speed over time). */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	UCurveFloat* EngineAccelerationCurve;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	UCurveFloat* EngineDecelerationCurve; 

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	float AirTorqueAcceleration;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	float TorqueAcceleration;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, meta = (EditCondition = "!bOrientRotationToMovementInAir"))
	float AirStrafeAcceleration;

	/** Vehicle moving force. */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	float VehicleAcceleration;

	/** Acceleration when boosting. */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	float VehicleBoostAcceleration;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	float BrakinDeceleration;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	UCurveFloat* SteeringActionCurve;

	/* This property scales the side velocity to substract to the forward velocity when the kart steers. 
	Set to 0 for no deceleration at all, Set to 1 to decelerate only relative to the side velocity of the kart.*/
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	float SteeringDecelerationSideVelocityFactorScale;

	/* It determines how fast the kart will decelerate to substract completely the scaled side velocity. */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	float SteeringDecelerationSideVelocityInterpolationSpeed;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	UCurveFloat* AirControlCurve;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	bool bTiltedThrottle;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	float TerminalSpeed;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	float LegalSpeedOffset;

	/* Slight delay to simulate swapping gears from forward to reverse. */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	float SwapGearDirectionDelay;

	/* A kart is considered idling when the HorizontalSpeed is 0 +- StopThreshold */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	float StopThreshold;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	FVector2D AccelerationCenterOfMassOffset;

	UPROPERTY(BlueprintReadOnly) // fixmevori: accessor
	TArray<FCachedSuspensionInfo> CachedSuspensionInfo;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	bool bOrientRotationToMovementInAir;

public:

#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
	virtual void PreSave(const class ITargetPlatform* TargetPlatform) override;
	virtual void OnConstruction(const FTransform& Transform) override;
	void UpdateHandlersTransformCDO();
#endif // WITH_EDITOR

};
