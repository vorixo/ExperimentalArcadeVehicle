// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "Kismet/KismetSystemLibrary.h"
#include "Engine/Public/DrawDebugHelpers.h"
#include "AVBaseVehicle.generated.h"

// Suspension
#define BACK_RIGHT 0
#define FRONT_RIGHT 1
#define FRONT_LEFT 2
#define BACK_LEFT 3
#define NUMBER_OF_WHEELS 4

// Misc vars
#define DEFAULT_GROUND_FRICTION 1
#define DEFAULT_GROUND_RESISTANCE 1
#define TERMINAL_VELOCITY_PREEMPTION_FORCE_OFFSET 2000.f
#define ORIENT_ROTATION_VELOCITY_MAX_RATE 3.f


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
	uint8 bOverRollForceThreshold : 1;

	UPROPERTY()
	float GroundFriction;

	UPROPERTY()
	float GroundResistance;

	FSuspensionHitInfo() :
		bWheelOnGround(false),
		bTraceHit(false),
		bOverRollForceThreshold(true),
		GroundFriction(DEFAULT_GROUND_FRICTION),
		GroundResistance(DEFAULT_GROUND_RESISTANCE)
	{

	}
};

USTRUCT(BlueprintType)
struct ARCADEVEHICLE_API FSuspensionData
{
	GENERATED_USTRUCT_BODY()

public:

	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly)
	float SuspensionLength;

	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly)
	float SuspensionStiffness;

	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly)
	float SuspensionDampForce;

	/* Trace Half Size */
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly)
	FVector2D TraceHalfSize;

	FSuspensionData() :
		SuspensionLength(60.f),
		SuspensionStiffness(4.f),
		SuspensionDampForce(1250.f),
		TraceHalfSize(30.f, 30.f)
	{

	}

	FSuspensionData(float inSuspensionLength, float inSuspensionStiffness, float inSuspensionDampForce, FVector2D inTraceHalfSize) :
		SuspensionLength(inSuspensionLength),
		SuspensionStiffness(inSuspensionStiffness),
		SuspensionDampForce(inSuspensionDampForce),
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

	UPROPERTY(BlueprintReadOnly)
	float SuspensionRatio;

	UPROPERTY(BlueprintReadOnly)
	FSuspensionData SuspensionData;

	FCachedSuspensionInfo() :
		ImpactNormal(FVector::ZeroVector),
		SuspensionRatio(0.f),
		SuspensionData(FSuspensionData())
	{

	}
};


USTRUCT(BlueprintType)
struct ARCADEVEHICLE_API FBasedPlatformInfo
{
	GENERATED_USTRUCT_BODY()
	
	/** Component we are based on */
	UPROPERTY(BlueprintReadOnly)
	UPrimitiveComponent* MovementBase;

	/** Location relative to MovementBase. Only valid if HasRelativeLocation() is true. */
	UPROPERTY(BlueprintReadOnly)
	FVector_NetQuantize100 Location;

	/** Rotation: relative to MovementBase if HasRelativeRotation() is true, absolute otherwise. */
	UPROPERTY(BlueprintReadOnly)
	FQuat Rotation;

};


UENUM(BlueprintType)
enum class EBasedPlatformSetup : uint8 {
	IgnoreBasedMovement		= 0		UMETA(DisplayName = "Ignore Based Movement"),
	IgnoreBasedRotation		= 1		UMETA(DisplayName = "Ignore Based Rotation"),
	ApplyBasedMovement		= 2		UMETA(DisplayName = "Apply Based Movement"),
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

	// Delegate for physics step
	FDelegateHandle OnPhysSceneStepHandle;
	void PhysSceneStep(FPhysScene* PhysScene, float DeltaTime);

	/* This event is called on every physics tick, including sub-steps. */
	void PhysicsTick(float SubstepDeltaTime);

	/**
	/* Returns true if the wheel is within the suspension distance threshold (meaning it is in the ground)                                                                     
	**/
	UFUNCTION()
	FSuspensionHitInfo CalcSuspension(FVector HoverComponentOffset, FCachedSuspensionInfo &InCachedInfo);

	UFUNCTION()
	virtual void ApplySuspensionForces();

	UFUNCTION(BlueprintNativeEvent, meta = (DisplayName = "TraceFunc"))
	bool TraceFunc(FVector Start, FVector End, FVector2D HalfSize, EDrawDebugTrace::Type DrawDebugType, FHitResult& OutHit);

	UFUNCTION()
	FVector GetOffsetedCenterOfVehicle() const;

	UFUNCTION()
	void SetThrottleInput(float InputAxis);

	UFUNCTION()
	void SetSteeringInput(float InputAxis);

	UFUNCTION()
	void SetBrakeInput(float InputAxis);

	UFUNCTION()
	virtual void ApplyInputStack(float DeltaTime);

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

	/** This function gets called when bIsMovingOnGround becomes true **/
	virtual void Landed(const FVector& HitNormal);

	UFUNCTION(BlueprintImplementableEvent)
	void OnLanded(const FVector &HitNormal);

	/** Computes the delta movement to apply on moving platforms **/
	UFUNCTION()
	void ComputeBasedMovement();

protected:
	// Reference to MMTPawn root component
	UPROPERTY()
	UPrimitiveComponent* PawnRootComponent;

	FBodyInstance* RootBodyInstance;
	static FBodyInstance* GetBodyInstance(UPrimitiveComponent* PrimitiveComponent);

	UPROPERTY(BlueprintReadOnly)
	FVector CurrentHorizontalVelocity;

	UPROPERTY(BlueprintReadOnly)
	FVector LocalVelocity;
	
	UPROPERTY(BlueprintReadOnly)
	float CurrentHorizontalSpeed;

	UPROPERTY(BlueprintReadOnly)
	FVector CurrentAngularVelocity;

	UPROPERTY(BlueprintReadOnly)
	float CurrentAngularSpeed;

	UPROPERTY(BlueprintReadWrite)
	float ScalarFrictionVal;

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

	UPROPERTY(BlueprintReadOnly)
	uint8 bOverRollForceThreshold : 1;

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
	
	UPROPERTY(BlueprintReadOnly)
	FBasedPlatformInfo BasedPlatformInfo;

	/** 1: Max influence 0: Min Influence **/
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, meta = (EditCondition = "bOrientRotationToMovementInAir"))
	float OrientRotationToMovementInAirInfluenceRate;

	/** 1: Max influence 0: Min Influence **/
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	float AccelerationInfluenceRateWhileBraking;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	bool bStickyWheels;

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

	// These four properties are really relevant: for visualization: Category = HoverComponent, VisibleAnywhere
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

	/* Decides on how the vehicle will behave on top of moving or rotating platforms. */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	EBasedPlatformSetup BasedMovementSetup;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	float GravityAir;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	float GravityGround;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	float GroundFriction;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	float GroundDetectionDistanceThreshold;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	float AntiRollForcedDistanceThreshold;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	float LinearDampingAir;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	float AngularDampingGround;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	float MaxSpeedBoosting;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	UCurveFloat* EngineAccelerationCurve;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	UCurveFloat* EngineDecelerationCurve; 

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	float TorqueSpeed;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	float AirYawSpeed;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, meta = (EditCondition = "!bOrientRotationToMovementInAir"))
	float AirStrafeSpeed;

	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	float VehicleAcceleration;

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
