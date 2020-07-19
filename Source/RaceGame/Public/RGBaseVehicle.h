// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "Kismet/KismetSystemLibrary.h"
#include "Engine/Public/DrawDebugHelpers.h"
#include "RGBaseVehicle.generated.h"

UCLASS()
class RACEGAME_API ARGBaseVehicle : public APawn
{
	GENERATED_BODY()

public:
	// Sets default values for this pawn's properties
	ARGBaseVehicle();

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

	UFUNCTION()
	bool CalcSuspension(FVector HoverComponentOffset, FVector &ImpactPoint, FVector &ImpactNormal);

	UFUNCTION()
	void ApplySuspensionForces();

	UFUNCTION(BlueprintNativeEvent, meta = (DisplayName = "TraceFunc"))
	bool TraceFunc(FVector Start, FVector End, EDrawDebugTrace::Type DrawDebugType, FHitResult& OutHit);

	UFUNCTION()
	void SetDampingBasedOnGroundInfo();

	UFUNCTION()
	FVector GetOffsetedCenterOfVehicle() const;

	UFUNCTION()
	void SetThrottleInput(float InputAxis);

	UFUNCTION()
	void SetSteeringInput(float InputAxis);

	UFUNCTION()
	void SetBrakeInput(float InputAxis);

	UFUNCTION()
	void ApplyInputStack();

	UFUNCTION(BlueprintPure)
	float getMaxSpeed() const;

	UFUNCTION(BlueprintPure)
	float getMaxAcceleration() const;

	UFUNCTION(BlueprintCallable)
	void SetBoosting(bool inBoost);

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
	TArray<FVector> ImpactPoints;

	UPROPERTY()
	TArray<FVector> ImpactNormals;

	UPROPERTY()
	float SuspensionRatio;

	UPROPERTY()
	float ScalarFrictionVal;

	UPROPERTY()
	FVector AvgedNormals;

	UPROPERTY()
	bool bIsMovingOnGround;

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
	FVector LastUpdateVelocity;

	UPROPERTY()
	bool bIsBoosting;

public:

	UPROPERTY(Category = HoverComponent, VisibleAnywhere, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	UStaticMeshComponent* CollisionMesh;

	UPROPERTY(Category = HoverComponent, VisibleAnywhere, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	FVector BackRight;

	UPROPERTY(Category = HoverComponent, VisibleAnywhere, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	FVector FrontRight;

	UPROPERTY(Category = HoverComponent, VisibleAnywhere, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	FVector FrontLeft;

	UPROPERTY(Category = HoverComponent, VisibleAnywhere, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	FVector BackLeft;
	
	UPROPERTY(EditDefaultsOnly)
	float SuspensionLength;

	UPROPERTY(EditDefaultsOnly)
	float SuspensionStiffness;

	UPROPERTY(EditDefaultsOnly)
	float SuspensionDampForce;

	UPROPERTY(EditDefaultsOnly)
	float GravityAir;

	UPROPERTY(EditDefaultsOnly)
	float GravityGround;

	UPROPERTY(EditDefaultsOnly)
	float GroundFriction;

	UPROPERTY(EditDefaultsOnly)
	bool bStickyWheels;

	UPROPERTY(EditDefaultsOnly)
	float PredictiveLandingThresholdDistance;

	UPROPERTY(EditDefaultsOnly)
	float LinearDampingGround;

	UPROPERTY(EditDefaultsOnly)
	float LinearDampingAir;

	UPROPERTY(EditDefaultsOnly)
	float AngularDampingGround;

	UPROPERTY(EditDefaultsOnly)
	float AngularDampingAir;

	UPROPERTY(EditDefaultsOnly)
	float EngineDecceleration;

	UPROPERTY(EditDefaultsOnly)
	float MaxSpeed;

	UPROPERTY(EditDefaultsOnly)
	float MaxSpeedBoosting;

	UPROPERTY(EditDefaultsOnly)
	float MaxBackwardsSpeed;

	UPROPERTY(EditDefaultsOnly)
	float TorqueSpeed;

	UPROPERTY(EditDefaultsOnly)
	float VehicleAcceleration;

	UPROPERTY(EditDefaultsOnly)
	float VehicleBoostAcceleration;

	UPROPERTY(EditDefaultsOnly)
	float BrakinDeceleration;

	UPROPERTY(EditDefaultsOnly)
	float BackwardsAcceleration;

	UPROPERTY(EditDefaultsOnly)
	UCurveFloat* EngineDeccelerationCurve;

	UPROPERTY(EditDefaultsOnly)
	UCurveFloat* SteeringActionCurve;

	UPROPERTY(EditDefaultsOnly)
	bool bTiltedThrottle;

	UPROPERTY(EditDefaultsOnly)
	FVector2D AccelerationCenterOfMassOffset;
};
