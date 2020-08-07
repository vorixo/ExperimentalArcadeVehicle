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
#define REPULSIVE_FORCE_MAX_WALKABLE_ANGLE 1000
#define ANTI_ROLL_FORCE 20000
#define TERMINAL_VELOCITY_PREEMPTION_FORCE_OFFSET 2000.f

#define PRINT_TICK(x) UKismetSystemLibrary::PrintString(this,x,true,false,FLinearColor::Red, 0.f)

#if WITH_EDITOR
class UArrowComponent;
#endif

USTRUCT()
struct ARCADEVEHICLE_API FSuspensionHitInfo
{
	GENERATED_USTRUCT_BODY()

public:

	UPROPERTY()
	bool bWheelOnGround;

	UPROPERTY()
	bool bTraceHit;

	UPROPERTY()
	float GroundFriction;

	UPROPERTY()
	float GroundResistance;

	FSuspensionHitInfo() : 
		bWheelOnGround(false),
		bTraceHit(false),
		GroundFriction(1.0f),
		GroundResistance(0.0f)
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

	UPROPERTY(EditDefaultsOnly)
	float SuspensionStiffness;

	UPROPERTY(EditDefaultsOnly)
	float SuspensionDampForce;

	FSuspensionData() :
		SuspensionLength(60.f),
		SuspensionStiffness(4.f),
		SuspensionDampForce(1250.f)
	{

	}
};

USTRUCT(BlueprintType)
struct ARCADEVEHICLE_API FCachedSuspensionInfo
{
	GENERATED_USTRUCT_BODY()

public:

	UPROPERTY()
	FVector ImpactPoint;

	UPROPERTY()
	FVector ImpactNormal;

	UPROPERTY(BlueprintReadOnly)
	float SuspensionRatio;

	UPROPERTY(BlueprintReadOnly)
	FSuspensionData SuspensionData;

	FCachedSuspensionInfo() :
		ImpactPoint(FVector::ZeroVector),
		ImpactNormal(FVector::ZeroVector),
		SuspensionRatio(0.f),
		SuspensionData(FSuspensionData())
	{

	}
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
	void ApplySuspensionForces();

	UFUNCTION(BlueprintNativeEvent, meta = (DisplayName = "TraceFunc"))
	bool TraceFunc(FVector Start, FVector End, EDrawDebugTrace::Type DrawDebugType, FHitResult& OutHit);

	UFUNCTION()
	FVector GetOffsetedCenterOfVehicle() const;

	UFUNCTION()
	void SetThrottleInput(float InputAxis);

	UFUNCTION()
	void SetSteeringInput(float InputAxis);

	UFUNCTION()
	void SetBrakeInput(float InputAxis);

	UFUNCTION()
	void ApplyInputStack(float DeltaTime);

	/** Gets you whatever max speed is being used*/
	UFUNCTION(BlueprintPure)
	float GetMaxSpeedAxisIndependent() const;

	/** Computes the current forward speed based on the acceleration curve */
	UFUNCTION(BlueprintPure)
	float GetComputedSpeed() const;

	UFUNCTION(BlueprintPure)
	float getMaxSpeed() const;

	UFUNCTION(BlueprintPure)
	float GetComputedBackwardsSpeed() const;

	/** Gets you whatever max speed is being used*/
	UFUNCTION(BlueprintPure)
	float GetComputedSpeedAxisIndependent() const;

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
	void ApplyGravityForce(float DeltaTime);

	UFUNCTION(BlueprintPure)
	float GetTerminalSpeed() const;

	/** Get the max angle in degrees of a walkable surface for the character. */
	UFUNCTION(BlueprintPure)
	float GetWalkableFloorAngle() const;

	/** Set the max angle in degrees of a walkable surface for the character. Also computes WalkableFloorZ. */
	UFUNCTION(BlueprintCallable)
	void SetWalkableFloorAngle(float InWalkableFloorAngle);

	/** Get the Z component of the normal of the steepest walkable surface for the character. Any lower than this and it is not walkable. */
	UFUNCTION()
	float GetWalkableFloorZ() const;

	/** Set the Z component of the normal of the steepest walkable surface for the character. Also computes WalkableFloorAngle. */
	UFUNCTION(BlueprintCallable)
	void SetWalkableFloorZ(float InWalkableFloorZ);

	/** Is any brakin force acting over the vehicle? */
	UFUNCTION(BlueprintPure)
	bool IsBraking() const;

protected:
	// Reference to MMTPawn root component
	UPROPERTY()
	UPrimitiveComponent* PawnRootComponent;

	FBodyInstance* RootBodyInstance;
	static FBodyInstance* GetBodyInstance(UPrimitiveComponent* PrimitiveComponent);

	UPROPERTY()
	FVector CurrentHorizontalVelocity;
	
	UPROPERTY()
	float CurrentHorizontalSpeed;

	UPROPERTY()
	FVector CurrentAngularVelocity;

	UPROPERTY()
	float CurrentAngularSpeed;

	UPROPERTY()
	float ScalarFrictionVal;

	UPROPERTY()
	FVector AvgedNormals;

	/* It's true if at least two wheels are touching the ground */
	UPROPERTY()
	bool bIsMovingOnGround;

	/* It's true if any wheel is touching the ground */
	UPROPERTY()
	bool bCompletelyInTheAir;

	UPROPERTY()
	bool bIsCloseToGround;

	UPROPERTY()
	float CurrentThrottleAxis;

	UPROPERTY()
	float CurrentSteeringAxis;

	UPROPERTY()
	float CurrentBrakeAxis;

	UPROPERTY()
	FVector ThrottleForce;

	UPROPERTY()
	FVector SteeringForce;

	UPROPERTY()
	FVector RGForwardVector;

	UPROPERTY()
	FVector RGUpVector;

	UPROPERTY()
	FVector RGRightVector;

	UPROPERTY()
	FVector RGLocation;

	UPROPERTY()
	FTransform RGWorldTransform;

	UPROPERTY()
	FVector LastUpdateForce;

	UPROPERTY()
	bool bIsBoosting;

	UPROPERTY()
	float CurrentGroundFriction;

	/** Resistance imposed by the current ground in which the user is navigating, will be translated to ground linear damping. */
	UPROPERTY()
	float CurrentGroundScalarResistance;

	UPROPERTY()
	float AccelerationAccumulatedTime;

	UPROPERTY()
	float MaxAccelerationCurveTime;

	UPROPERTY()
	float MaxSpeed;

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
	UPROPERTY()
	FVector BackRight;

	UPROPERTY()
	FVector FrontRight;

	UPROPERTY()
	FVector FrontLeft;

	UPROPERTY()
	FVector BackLeft;
	// These will spawn the traces

	UPROPERTY(EditDefaultsOnly, Category = SuspensionFront)
	FSuspensionData SuspensionFront;

	UPROPERTY(EditDefaultsOnly, Category = SuspensionRear)
	FSuspensionData SuspensionRear;

	UPROPERTY(EditDefaultsOnly)
	float GravityAir;

	UPROPERTY(EditDefaultsOnly)
	float GravityGround;

	UPROPERTY(EditDefaultsOnly)
	float GroundFriction;

	UPROPERTY(EditDefaultsOnly)
	bool bStickyWheels;

	UPROPERTY(EditDefaultsOnly)
	float StickyWheelsGroundDistanceThreshold;

	UPROPERTY(EditDefaultsOnly)
	float LinearDampingAir;

	UPROPERTY(EditDefaultsOnly)
	float AngularDampingGround;

	UPROPERTY(EditDefaultsOnly, meta = (ClampMin = "1.0", ClampMax = "100.0", UIMin = "1.0", UIMax = "100.0"))
	float AngularDampingAir;

	UPROPERTY(EditDefaultsOnly)
	float MaxSpeedBoosting;

	UPROPERTY(EditDefaultsOnly)
	UCurveFloat* EngineAccelerationCurve;

	UPROPERTY(EditDefaultsOnly)
	float MaxBackwardsSpeed;

	UPROPERTY(EditDefaultsOnly)
	float TorqueSpeed;

	UPROPERTY(EditDefaultsOnly)
	float VehicleAcceleration;

	UPROPERTY(EditDefaultsOnly)
	float EngineDecceleration;

	UPROPERTY(EditDefaultsOnly)
	float VehicleBoostAcceleration;

	UPROPERTY(EditDefaultsOnly)
	float BrakinDeceleration;

	UPROPERTY(EditDefaultsOnly)
	float BackwardsAcceleration;

	UPROPERTY(EditDefaultsOnly)
	UCurveFloat* SteeringActionCurve;

	UPROPERTY(EditDefaultsOnly)
	bool bTiltedThrottle;

	UPROPERTY(EditDefaultsOnly)
	float TerminalSpeed;

	UPROPERTY(EditDefaultsOnly)
	float LegalSpeedOffset;

	UPROPERTY(EditDefaultsOnly)
	FVector2D AccelerationCenterOfMassOffset;

	UPROPERTY(BlueprintReadOnly) // fixmevori: accessor
	TArray<FCachedSuspensionInfo> CachedSuspensionInfo;

private:
	/**
	* Max angle in degrees of a walkable surface. Any greater than this and it is too steep to be walkable.
	*/
	UPROPERTY(EditDefaultsOnly, meta = (ClampMin = "0.0", ClampMax = "90.0", UIMin = "0.0", UIMax = "90.0"))
	float WalkableFloorAngle;

	/**
	 * Minimum Z value for floor normal. If less, not a walkable surface. Computed from WalkableFloorAngle.
	 */
	UPROPERTY(VisibleAnywhere)
	float WalkableFloorZ;

public:

#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
	virtual void PreSave(const class ITargetPlatform* TargetPlatform) override;
	virtual void OnConstruction(const FTransform& Transform) override;
	void UpdateHandlersTransformCDO();
#endif // WITH_EDITOR

};
